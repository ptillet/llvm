//===-- llvm/lib/Target/NVPTX/TLVMBindLayout.h -------------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the TLVM layout information for
// NVIDIA hardware
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_NVPTX_TLVMBINDLAYOUT_H
#define LLVM_LIB_TARGET_NVPTX_TLVMBINDLAYOUT_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/DenseMap.h"

namespace llvm {
class FunctionPass;
class Value;
class Function;
class User;
class CallInst;

class Axis{
public:
  Axis(unsigned WS, unsigned NW, unsigned PT)
    : WarpSize(WS), NumWarps(NW), PerThread(PT),
     BlockSize(WS*NW), TileSize(WS*NW*PT){
  }

  unsigned WarpSize;
  unsigned NumWarps;
  unsigned PerThread;
  unsigned BlockSize;
  unsigned TileSize;
};

class TLVMLayout{
  typedef SmallVector<Axis*, 4> AxesType;

public:
  template<typename ... T>
  TLVMLayout(T... Axs): Axes({Axs...}){ }

  unsigned numAxes() const { return Axes.size(); }
  AxesType::const_iterator axis_begin() const { return Axes.begin(); }
  AxesType::const_iterator axis_end() const { return Axes.end(); }

private:
  AxesType Axes;
};

// Layout binding pass
class TLVMBindLayout: public FunctionPass {
  llvm::DenseMap<Value*, TLVMLayout> Layouts;
  llvm::SmallVector<Axis*, 16> AxesPool;

private:
  TLVMLayout* getOperandLayout(User *I, unsigned Idx);
  Axis* getOperandAxis(User *I, unsigned Idx, unsigned Ax);
  Axis* makeAxis(unsigned WarpSize, unsigned NumWarp, unsigned Repeat);
  void handleIntrinsic(CallInst *Call, Function *Callee);

public:
  static char ID;
  TLVMBindLayout();
  ~TLVMBindLayout();

  bool runOnFunction(Function &F) override;
  TLVMLayout *get(Value *I);
};

}

#endif
