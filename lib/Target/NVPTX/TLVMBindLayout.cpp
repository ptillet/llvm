//===- TLVMBindLayout.cpp - TLVM Bind Layout -------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This pass attributes a layout to each Tile and Range present in the program
// and checks the correctness of each operation. Here, a layout describes
// which threads/warp own which element of a particular tile.
//
//===----------------------------------------------------------------------===//

#include "NVPTX.h"
#include "TLVMBindLayout.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Pass.h"

#define TLVM_BIND_LAYOUT_FUNCTION "__tlvm_bind_layout"
#define DEBUG_TYPE "tlvm-bind-layout"


namespace llvm {

// Constructor
TLVMBindLayout::TLVMBindLayout() : FunctionPass(ID) { }

// Destructor
TLVMBindLayout::~TLVMBindLayout(){
  // Clean up pool
  std::for_each(AxesPool.begin(), AxesPool.end(), std::default_delete<Axis>());
}

TLVMLayout* TLVMBindLayout::getOperandLayout(User *I, unsigned Idx){
  auto It = Layouts.find(I->getOperand(Idx));
  assert(It != Layouts.end() && "Layout for operand does not exist");
  return &It->second;
}

Axis* TLVMBindLayout::getOperandAxis(User *I, unsigned Idx, unsigned Ax){
  return *(getOperandLayout(I, Idx)->axis_begin() + Ax);
}

Axis* TLVMBindLayout::makeAxis(unsigned WarpSize, unsigned NumWarp, unsigned Repeat){
  AxesPool.push_back(new Axis(WarpSize, NumWarp, Repeat));
  return AxesPool.back();
}

void TLVMBindLayout::handleIntrinsic(CallInst *Call, Function *Callee){
  switch (Callee->getIntrinsicID()) {
  // Slice
  case Intrinsic::tlvm_read_slice_x:{
    Axis* Ax = makeAxis(32, 2, 2);
    Layouts.insert({Call, {Ax}});
    break;
  }

  // GetTilePtr
  case Intrinsic::tlvm_gtp_1d:{
    Axis* Ax = getOperandAxis(Call, 1, 0);
    Layouts.insert({Call, {Ax}});
    break;
  }

  default:
    break;
  }
}

bool TLVMBindLayout::runOnFunction(Function &F){
  for(Function::iterator::value_type &BB: F){
  for(BasicBlock::iterator::value_type &I : BB){
    // Function call
    if(CallInst *Call = dyn_cast<CallInst>(&I))
    if(Function *Callee = Call->getCalledFunction())
      handleIntrinsic(Call, Callee);

    // Load
    if(LoadInst *Load = dyn_cast<LoadInst>(&I)){
      Layouts.insert({Load, *getOperandLayout(Load, 0)}).second;
    }

    // Binary
    if(BinaryOperator *BinOp = dyn_cast<BinaryOperator>(&I))
      Layouts.insert({BinOp, *getOperandLayout(BinOp, 0)}).second;
  }
  }
  return false;
}

TLVMLayout * TLVMBindLayout::get(Value *I){
  auto It = Layouts.find(I);
  if(It != Layouts.end())
    return &It->second;
  return nullptr;
}

char TLVMBindLayout::ID = 1;
}

// Initialization
namespace llvm {
void initializeTLVMBindLayoutPass(PassRegistry &);

FunctionPass *createTLVMBindLayoutPass() {
  return new TLVMBindLayout();
}
}

using namespace llvm;
INITIALIZE_PASS(TLVMBindLayout, "tlvm-bind-layout",
                "Bind layout value to each tile type (TLVM)", false, false)
