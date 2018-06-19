let TargetPrefix = "tlvm" in {

  def int_tlvm_read_slice_x     
    : Intrinsic<[llvm_slice_ty], [], [IntrNoMem], 
                "llvm.tlvm.read.slice.x">;
  
  def int_tlvm_gtp_1d
    : Intrinsic<[llvm_anyptr_ty],
                [llvm_anyptr_ty, llvm_slice_ty],
                [IntrNoMem],
                "llvm.tlvm.gtp.1d">;
  
}
