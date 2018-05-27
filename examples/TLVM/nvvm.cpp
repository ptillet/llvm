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

    // Function
    llvm::Type* int32_t = llvm::Type::getInt32Ty(context);
    llvm::Type* tile_t = llvm::TileType::get(int32_t, 1);
    llvm::Type* tensor_t = llvm::TensorType::get(tile_t);
    llvm::Type* range_t = llvm::Type::getRangeTy(context);

    llvm::FunctionType* prototype = llvm::FunctionType::get(llvm::Type::getVoidTy(context), std::vector<llvm::Type*>{tensor_t}, false);
    llvm::Function* F = llvm::Function::Create(prototype, llvm::Function::ExternalLinkage, "kernel", module.get());
    std::vector<llvm::Value*> arguments;
    std::transform(F->arg_begin(), F->arg_end(), std::back_inserter(arguments), [&](llvm::Argument& x){ return &x;});
    llvm::BasicBlock* block = llvm::BasicBlock::Create(context, "entry", F);
    builder.SetInsertPoint(block);
    llvm::Value* range = builder.CreateCall(llvm::Intrinsic::getDeclaration(module.get(), llvm::Intrinsic::tlvm_read_range_x));
    llvm::Value* x = builder.CreateLoad(arguments[0]);
    std::cout << range->getType()->getTypeID() << std::endl;
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

    auto machine = target->createTargetMachine(module->getTargetTriple(), "sm_52", "", llvm::TargetOptions(), llvm::Reloc::Model());
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
