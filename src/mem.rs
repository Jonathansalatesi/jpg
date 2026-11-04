use std::arch::naked_asm;

#[cfg(all(target_arch = "x86_64", target_os = "windows"))]
#[unsafe(naked)]
pub extern "C" fn memcpy(src: *const u8, dst: *mut u8, dstlen: u64) {
    naked_asm!(
        "pushfq",
        "push rsi",
        "push rdi",
        "cld",
        "mov rsi, rcx",
        "mov rdi, rdx",
        "mov rcx, r8",
        "rep movsb",
        "pop rdi",
        "pop rsi",
        "popfq",
        "ret"
    )
}