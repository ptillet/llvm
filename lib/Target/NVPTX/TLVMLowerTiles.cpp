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
#include "llvm/Support/raw_ostream.h"

#define TLVM_BIND_LAYOUT_FUNCTION "__tlvm_lower_tiles"
#define DEBUG_TYPE "tlvm-lower-tiles"

namespace llvm {

// Iteration over range
template<class IteratorTy>
void dynloop(IteratorTy bound_begin, IteratorTy bound_end, std::function<void(ArrayRef<size_t>)> doWork){
  size_t depth = std::distance(bound_begin, bound_end);
  std::vector<size_t> Idx(depth, 0);
  size_t i = 0;
  while(true){
    doWork(ArrayRef<size_t>(Idx.data(), Idx.size()));
    Idx[0]++;
    while(Idx[i] == *(bound_begin + i)){
      if(i == depth - 1)
        return;
      Idx[i++] = 0;
      Idx[i]++;
    }
  }
}


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

  SmallVector<unsigned, 4>::const_iterator perthread_begin() const{
    return PerThread.begin();
  }

  SmallVector<unsigned, 4>::const_iterator perthread_end() const{
    return PerThread.end();
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
  LoweredTypeImpl *makeImpl(Value *V, const TLVMLayout *L);

  bool lowerSliceIntrinsic(Instruction *I, Intrinsic::ID ID, LoweredSlice *Impl, llvm::IRBuilder<> &Builder);
  bool lowerTileIntrinsic(Instruction *I, Intrinsic::ID ID, LoweredTile *Impl, llvm::IRBuilder<> &Builder);
  bool lowerLoad(LoadInst *I, LoweredTile *Impl, llvm::IRBuilder<> &Builder);
  bool lowerStore(StoreInst *I, llvm::IRBuilder<> &Builder);
  bool lowerBinary(BinaryOperator *I, LoweredTile *Impl, llvm::IRBuilder<> &Builder);

public:
  static char ID;

  TLVMLowerTiles(): FunctionPass(ID){ }

  void getAnalysisUsage(AnalysisUsage & AU) const;
  bool runOnFunction(Function &F) override;
};


LoweredTypeImpl *TLVMLowerTiles::makeImpl(Value *V, const TLVMLayout *L){
  Type *Ty = V->getType();
  Type::TypeID TyID = Ty->getTypeID();
  if(Ty->isPointerTy())
    TyID = Ty->getPointerElementType()->getTypeID();
  switch(TyID){
  case llvm::Type::TileTyID:
    return Impls.insert({V, new LoweredTile(*L)}).first->second;
  case llvm::Type::SliceTyID:
    return Impls.insert({V, new LoweredSlice(*L)}).first->second;
  default:
    return nullptr;
  }
}

void TLVMLowerTiles::getAnalysisUsage(AnalysisUsage &AU) const{
  AU.addRequired<TLVMBindLayout>();
  AU.setPreservesAll();
}

bool TLVMLowerTiles::lowerSliceIntrinsic(Instruction *I, Intrinsic::ID ID,
                                        LoweredSlice *Impl, llvm::IRBuilder<> &Builder){
  switch(ID){
  // Read Slice
  case Intrinsic::tlvm_read_slice_x: {
    Function *read_tid = llvm::Intrinsic::getDeclaration(I->getModule(),
                               llvm::Intrinsic::nvvm_read_ptx_sreg_tid_x);
    Impl->setStart(Builder.CreateCall(read_tid, {}));
    return true;
  }

  default:
    return false;
  }
}

bool TLVMLowerTiles::lowerTileIntrinsic(Instruction *I, Intrinsic::ID ID,
                                        LoweredTile *Impl, llvm::IRBuilder<> &Builder){
  switch(ID){
  // GetTilePtr
  case Intrinsic::tlvm_gtp_1d:{
    Value *Ptr = I->getOperand(0);
    Value *Slice = I->getOperand(1);
    Value *Start = static_cast<LoweredSlice*>(Impls.lookup(Slice))->getStart();
    dynloop(Impl->perthread_begin(), Impl->perthread_end(),
            [&](ArrayRef<size_t> Idx){
      Value *_Idx = ConstantInt::get(Type::getInt32Ty(I->getContext()), Idx[0]);
      Impl->setValue(Idx, Builder.CreateGEP(Ptr, Builder.CreateAdd(Start, _Idx)));
    });
    return true;
  }

  default:
    return false;
  }
}

bool TLVMLowerTiles::lowerLoad(LoadInst *I, LoweredTile *Impl, llvm::IRBuilder<> &Builder){
  Value *Ptr = I->getPointerOperand();
  LoweredTile *PtrImpl = static_cast<LoweredTile *>(Impls.lookup(Ptr));
  dynloop(Impl->perthread_begin(), Impl->perthread_end(),
          [&](ArrayRef<size_t> Idx){
    Impl->setValue(Idx, Builder.CreateLoad(PtrImpl->getValue(Idx), I->isVolatile()));
  });
  return true;
}

bool TLVMLowerTiles::lowerStore(StoreInst *I, llvm::IRBuilder<> &Builder){
  Value *Ptr = I->getPointerOperand();
  Value *V = I->getValueOperand();
  LoweredTile *PtrImpl = static_cast<LoweredTile *>(Impls.lookup(Ptr));
  LoweredTile *VImpl = static_cast<LoweredTile *>(Impls.lookup(V));
  dynloop(VImpl->perthread_begin(), VImpl->perthread_end(),
          [&](ArrayRef<size_t> Idx){
    Builder.CreateStore(VImpl->getValue(Idx), PtrImpl->getValue(Idx), I->isVolatile());
  });
  return true;
}

bool TLVMLowerTiles::lowerBinary(BinaryOperator *I, LoweredTile *Impl, llvm::IRBuilder<> &Builder){
  Value *LHS = I->getOperand(0);
  Value *RHS = I->getOperand(1);
  LoweredTile *LImpl = static_cast<LoweredTile *>(Impls.lookup(LHS));
  LoweredTile *RImpl = static_cast<LoweredTile *>(Impls.lookup(RHS));
  dynloop(Impl->perthread_begin(), Impl->perthread_end(),
          [&](ArrayRef<size_t> Idx){
    Impl->setValue(Idx, Builder.CreateBinOp(I->getOpcode(), LImpl->getValue(Idx), RImpl->getValue(Idx)));
  });
  return true;
}


bool TLVMLowerTiles::runOnFunction(Function &F){
  TLVMBindLayout &Layouts = getAnalysis<TLVMBindLayout>();
  llvm::IRBuilder<> Builder(F.getContext());
  std::vector<Instruction *> toDelete;
  for(Function::iterator::value_type &BB: F)
  for(BasicBlock::iterator::value_type &I : BB){
    Builder.SetInsertPoint(&I);
    LoweredTypeImpl *Impl = makeImpl(&I, Layouts.get(&I));

    bool Lowered = false;
    // Lower intrinsics
    if(CallInst *Call = dyn_cast<CallInst>(&I)){
      Function *Callee = Call->getCalledFunction();
      Intrinsic::ID IntrID = Callee->getIntrinsicID();
      Lowered |= lowerSliceIntrinsic(Call, IntrID, (LoweredSlice *)Impl, Builder);
      Lowered |= lowerTileIntrinsic(Call, IntrID, (LoweredTile *)Impl, Builder);
    }
    if(LoadInst *Load = dyn_cast<LoadInst>(&I)){
      Lowered |= lowerLoad(Load, (LoweredTile *)Impl, Builder);
    }
    if(StoreInst *Store = dyn_cast<StoreInst>(&I)){
      Lowered |= lowerStore(Store, Builder);
    }
    if(BinaryOperator *BinOp = dyn_cast<BinaryOperator>(&I)){
      Lowered |= lowerBinary(BinOp, (LoweredTile *)Impl, Builder);
    }

    // Delete lowered instruction
    if(Lowered) toDelete.push_back(&I);
  }

  for(auto It = toDelete.rbegin(); It != toDelete.rend(); ++It)
    (*It)->eraseFromParent();

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
