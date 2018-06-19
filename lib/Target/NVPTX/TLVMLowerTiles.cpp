//===- TLVMLowerTile.cpp - TLVM Lower Tile ---------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
//===----------------------------------------------------------------------===//

#include "NVPTX.h"
#include <vector>
#include "TLVMBindLayout.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Pass.h"

#define TLVM_BIND_LAYOUT_FUNCTION "__tlvm_lower_tiles"
#define DEBUG_TYPE "tlvm-lower-tiles"

namespace llvm {

// Tile Lowering pass
class LoweredTypeImpl{

};

// Lowered Tile values
class LoweredTile: public LoweredTypeImpl {

  template<class IteratorTy1, class IteratorTy2>
  static unsigned LinearizeIdx(IteratorTy1 IdxBegin, IteratorTy1 IdxEnd,
                               IteratorTy2 StrideBegin){
    unsigned Res = 0;
    for(; IdxBegin != IdxEnd; ++IdxBegin, ++StrideBegin)
      Res += *IdxBegin * *StrideBegin;
    return Res;
  }

public:
  LoweredTile(const TLVMLayout &L)
   : Layout(L), PerThread(L.numAxes()), Strides(L.numAxes(), 1) {
    // Number of values per thread
    std::transform(L.axis_begin(), L.axis_end(), PerThread.begin(),
                   [](Axis* Ax){ return Ax->PerThread; });
    // Linearized values array
    Values.resize(std::accumulate(PerThread.begin(), PerThread.end(), 1,
                                  std::multiplies<int>()));
    // Strides in values array
    std::partial_sum(PerThread.begin(), PerThread.end() - 1, Strides.begin() + 1,
                     std::multiplies<int>());
  }

  void setValue(ArrayRef<size_t> Idx, Value *V){
    Values[LinearizeIdx(Idx.begin(), Idx.end(), Strides.begin())] = V;
  }

  Value *getValue(ArrayRef<size_t> Idx){
    return Values[LinearizeIdx(Idx.begin(), Idx.end(), Strides.begin())];
  }

  void forEach(std::function<Value *(size_t)> Fn){
    for(size_t i = 0; i < PerThread[0]; i++)
      setValue({i}, Fn(i));
  }

  void forEach(std::function<void (Value *)> Fn){
    for(size_t i = 0; i < PerThread[0]; i++)
      Fn(getValue({i}));
  }

  unsigned perThread(unsigned Idx){
    return PerThread[Idx];
  }

private:
  const TLVMLayout &Layout;
  SmallVector<Value *, 16> Values;
  SmallVector<unsigned, 4> PerThread;
  SmallVector<unsigned, 4> Strides;
};

// Lowered Slice values
class LoweredSlice: public LoweredTypeImpl{
public:
  LoweredSlice(const TLVMLayout &L)
    : Layout(L) { }

  void setStart(Value *S) { Start = S;}
  Value * getStart() { return Start; }

private:
  const TLVMLayout &Layout;
  Value *Start;
};

// Tiles lowering pass
class TLVMLowerTiles: public FunctionPass {
  DenseMap<Value *, LoweredTypeImpl *> Impls;
  LoweredTypeImpl *makeImpl(Value *V, const TLVMLayout &L);
  static inline bool isLowered(Value * V);

  void lowerSliceIntrinsic(Instruction *I, Intrinsic::ID ID, LoweredSlice *Impl, llvm::IRBuilder<> &Builder);
  void lowerTileIntrinsic(Instruction *I, Intrinsic::ID ID, LoweredTile *Impl, llvm::IRBuilder<> &Builder);
  void lowerLoad(LoadInst *I, LoweredTile *Impl, llvm::IRBuilder<> &Builder);
  void lowerStore(StoreInst *I, llvm::IRBuilder<> &Builder);
  void lowerInstruction(Instruction *I, TLVMBindLayout &Layouts, llvm::IRBuilder<> &Builder);

public:
  static char ID;

  TLVMLowerTiles(): FunctionPass(ID){ }

  void getAnalysisUsage(AnalysisUsage & AU) const;
  bool runOnFunction(Function &F) override;
};

bool TLVMLowerTiles::isLowered(Value * V){
  Type::TypeID TyID = V->getType()->getTypeID();
  return TyID == Type::TileTyID || TyID == Type::SliceTyID;
}

LoweredTypeImpl *TLVMLowerTiles::makeImpl(Value *V, const TLVMLayout &L){
  switch(V->getType()->getTypeID()){
  case llvm::Type::TileTyID:
    return Impls.insert({V, new LoweredTile(L)}).first->second;
  case llvm::Type::SliceTyID:
    return Impls.insert({V, new LoweredSlice(L)}).first->second;
  default:
    return nullptr;
  }
}

void TLVMLowerTiles::getAnalysisUsage(AnalysisUsage &AU) const{
  AU.addRequired<TLVMBindLayout>();
  AU.setPreservesAll();
}

void TLVMLowerTiles::lowerSliceIntrinsic(Instruction *I, Intrinsic::ID ID,
                                        LoweredSlice *Impl, llvm::IRBuilder<> &Builder){
  switch(ID){
  // Read Slice
  case Intrinsic::tlvm_read_slice_x: {
    Function *read_tid = llvm::Intrinsic::getDeclaration(I->getModule(),
                               llvm::Intrinsic::nvvm_read_ptx_sreg_tid_x);
    Impl->setStart(Builder.CreateCall(read_tid, {}));
    break;
  }

  default:
    break;
  }
}

void TLVMLowerTiles::lowerTileIntrinsic(Instruction *I, Intrinsic::ID ID,
                                        LoweredTile *Impl, llvm::IRBuilder<> &Builder){
  switch(ID){
  // GetTilePtr
  case Intrinsic::tlvm_gtp_1d:
    Impl->forEach([&](size_t Idx){
      Value *Ptr = I->getOperand(0);
      Value *Slice = I->getOperand(1);
      Value *Start = static_cast<LoweredSlice*>(Impls.lookup(Slice))->getStart();
      Value *_Idx = ConstantInt::get(Type::getInt32Ty(I->getContext()), Idx);
      Value *X = Builder.CreateGEP(Ptr, Builder.CreateAdd(Start, _Idx));
      return X;
    });
    break;

  default:
    break;
  }
}

void TLVMLowerTiles::lowerLoad(LoadInst *I, LoweredTile *Impl, llvm::IRBuilder<> &Builder){
  Impl->forEach([&](size_t Idx){
    Value *Ptr = I->getPointerOperand();
    LoweredTile *PtrImpl = static_cast<LoweredTile *>(Impls.lookup(Ptr));
    return Builder.CreateLoad(PtrImpl->getValue(Idx), I->isVolatile());
  });
}

void TLVMLowerTiles::lowerStore(StoreInst *I, llvm::IRBuilder<> &Builder){
  Value *Ptr = I->getPointerOperand();
  Value *V = I->getValueOperand();
  LoweredTile *PtrImpl = static_cast<LoweredTile *>(Impls.lookup(Ptr));
  LoweredTile *VImpl = static_cast<LoweredTile *>(Impls.lookup(V));
  for(unsigned Idx = 0; Idx < VImpl->perThread(0); ++Idx)
    Builder.CreateStore(VImpl->getValue(Idx), PtrImpl->getValue(Idx), I->isVolatile());
}

void TLVMLowerTiles::lowerInstruction(Instruction *I, TLVMBindLayout & Layouts, llvm::IRBuilder<> &Builder){
  if(!isLowered(I))
    return;
  Builder.SetInsertPoint(I);
  LoweredTypeImpl *Impl = makeImpl(I, Layouts.get(I));
  // Lower intrinsics
  if(CallInst *Call = dyn_cast<CallInst>(I)){
    Function *Callee = Call->getCalledFunction();
    Intrinsic::ID IntrID = Callee->getIntrinsicID();
    lowerSliceIntrinsic(Call, IntrID, (LoweredSlice *)Impl, Builder);
    lowerTileIntrinsic(Call, IntrID, (LoweredTile *)Impl, Builder);
  }
  // Load
  if(LoadInst *Load = dyn_cast<LoadInst>(I))
    lowerLoad(Load, (LoweredTile *)Impl, Builder);
  // Store
  if(StoreInst *Store = dyn_cast<StoreInst>(I))
    lowerStore(Store, Builder);
  // Replace uses
  for(auto &U: I->uses()){
    if(U.get() != I){
    printf("%d", U.get());
    lowerInstruction((Instruction *)U.get(), Layouts, Builder);
    }
  }
}


bool TLVMLowerTiles::runOnFunction(Function &F){
  TLVMBindLayout &Layouts = getAnalysis<TLVMBindLayout>();
  llvm::IRBuilder<> Builder(F.getContext());
  for(Function::iterator::value_type &BB: F)
  for(BasicBlock::iterator::value_type &I : BB)
    lowerInstruction(&I, Layouts, Builder);

  return true;
}


char TLVMLowerTiles::ID = 1;
}

// Initialization
namespace llvm {
void initializeTLVMLowerTilesPass(PassRegistry &);

FunctionPass *createTLVMLowerTilesPass() {
  return new TLVMLowerTiles();
}
}

using namespace llvm;
INITIALIZE_PASS(TLVMLowerTiles, "tlvm-lower-tiles",
                "Lower tiles to legal LLVM+NVPTX intrinsics (TLVM)", false, false)
