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

pub fn u16_be(arr: &[u8]) -> u16 {
    u16::from_be_bytes([arr[0], arr[1]])
}

#[inline]
pub fn divide_u8(v: u8) -> (u8, u8) {
    let h = (v & 0xf0) >> 4;
    let l = v & 0x0f;
    (h, l)
}