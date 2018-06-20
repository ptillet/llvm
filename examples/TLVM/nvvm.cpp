#include <iostream>

#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Type.h"
#include "llvm/IR/Verifier.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Host.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"


int main(){
    std::string error;

    llvm::InitializeAllTargetInfos();
    llvm::InitializeAllTargets();
    llvm::InitializeAllTargetMCs();
    llvm::InitializeAllAsmParsers();
    llvm::InitializeAllAsmPrinters();

    // Module
    llvm::LLVMContext context;
    auto module = llvm::make_unique<llvm::Module>("NVVM toy example", context);
    llvm::IRBuilder<> builder(context);

    // Types
    llvm::IntegerType* int32_t = llvm::Type::getInt32Ty(context);
    llvm::PointerType* int32_ptr_t = llvm::PointerType::get(int32_t, 0);
    llvm::Type* tile_t = llvm::TileType::get(int32_t, 1);
    llvm::PointerType* tile_ptr_t = llvm::PointerType::get(tile_t, 0);
    llvm::Function* read_slice_x = llvm::Intrinsic::getDeclaration(module.get(), llvm::Intrinsic::tlvm_read_slice_x);
    llvm::Function* gtp = llvm::Intrinsic::getDeclaration(module.get(), llvm::Intrinsic::tlvm_gtp_1d, {tile_ptr_t, int32_ptr_t});

    // Function
    llvm::FunctionType* prototype = llvm::FunctionType::get(llvm::Type::getVoidTy(context), std::vector<llvm::Type*>{int32_ptr_t}, false);
    llvm::Function* F = llvm::Function::Create(prototype, llvm::Function::ExternalLinkage, "kernel", module.get());
    std::vector<llvm::Value*> arguments;
    std::transform(F->arg_begin(), F->arg_end(), std::back_inserter(arguments), [&](llvm::Argument& x){ return &x;});
    // First basic block
    llvm::BasicBlock* block = llvm::BasicBlock::Create(context, "entry", F);
    builder.SetInsertPoint(block);
    llvm::Value* _1 = builder.CreateCall(read_slice_x);
    llvm::CallInst* _2 = builder.CreateCall(gtp, {arguments[0], _1});
    llvm::Value* _3 = builder.CreateLoad(_2);
    llvm::Value* _4 = builder.CreateAdd(_3, _3);
    builder.CreateStore(_4, _2);
    builder.CreateRet(NULL);


    // Set metadata
    llvm::Metadata *md_args[] = {
      llvm::ValueAsMetadata::get(F),
      llvm::MDString::get(context, "kernel"),
      llvm::ValueAsMetadata::get(llvm::ConstantInt::get(llvm::Type::getInt32Ty(context), 1))
    };
    module->getOrInsertNamedMetadata("nvvm.annotations")->addOperand(llvm::MDNode::get(context, md_args));

    // Machine
    module->setTargetTriple("nvptx64-nvidia-cuda");
    auto target = llvm::TargetRegistry::lookupTarget(module->getTargetTriple(), error);

    llvm::TargetMachine *machine = target->createTargetMachine(module->getTargetTriple(), "sm_52", "", llvm::TargetOptions(), llvm::Reloc::Model());
    module->setDataLayout(machine->createDataLayout());

    // Emit
    llvm::SmallVector<char, 0> buffer;
    llvm::raw_svector_ostream stream(buffer);

    llvm::legacy::PassManager pass;
    machine->addPassesToEmitFile(pass, stream, llvm::TargetMachine::CGFT_AssemblyFile);
    pass.run(*module);

    std::string sources(buffer.begin(), buffer.end());
    std::cout << sources << std::endl;
}

//    llvm::Function* fn = llvm::Intrinsic::getDeclaration(module.get(), llvm::Intrinsic::nvvm_read_ptx_sreg_tid_x);
//    llvm::BasicBlock* block = llvm::BasicBlock::Create(context, "entry", F);
//    builder.SetInsertPoint(block);
//    llvm::Value* _1 = llvm::ConstantInt::get(int32_t, 1);
//    llvm::Value* _tid = builder.CreateSExt(builder.CreateCall(fn), int32_t);
//    llvm::Value* _arg = builder.CreatePtrToInt(arguments[0], int32_t);
//    llvm::Value* _ptr = builder.CreateAdd(_arg, _tid);
//    llvm::PointerType* ptr_t = llvm::Type::getInt32PtrTy(context, 1);
//    llvm::Value* _addr = builder.CreateIntToPtr(_ptr, ptr_t);
//    builder.CreateStore(_1, _addr);
