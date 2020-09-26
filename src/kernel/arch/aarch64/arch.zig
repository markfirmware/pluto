const std = @import("std");
const vmm = @import("../../vmm.zig");
const mem = @import("../../mem.zig");
const Serial = @import("../../serial.zig").Serial;
const TTY = @import("../../tty.zig").TTY;
const log = @import("../../log.zig");
const rpi = @import("rpi.zig");
const mmio = @import("mmio.zig");
/// The type of the payload passed to a virtual memory mapper.
// TODO: implement
pub const VmmPayload = usize;
pub const BootPayload = *const rpi.RaspberryPiBoard;

// TODO: implement
pub const MEMORY_BLOCK_SIZE: usize = 4 * 1024;

// TODO: implement
pub const VMM_MAPPER: vmm.Mapper(VmmPayload) = vmm.Mapper(VmmPayload){ .mapFn = undefined, .unmapFn = undefined };

// TODO: implement
pub const KERNEL_VMM_PAYLOAD: VmmPayload = 0;

// The system clock frequency in Hz
const SYSTEM_CLOCK: usize = 700000000;

pub var mmio_addr: usize = undefined;

extern var KERNEL_PHYSADDR_START: *u32;
extern var KERNEL_PHYSADDR_END: *u32;

pub fn initTTY(boot_payload: BootPayload) TTY {
    return undefined;
}

pub fn initMmioAddress(board: *rpi.RaspberryPiBoard) void {
    mmio_addr = board.mmioAddress();
}

// The auxiliary uart (uart1) is the primary uart used for the console on pi3b and up.
//  The main uart (uart0) is used for the on-board bluetooth device on these boards.
//  See https://www.raspberrypi.org/documentation/configuration/uart.md
//  Note that config.txt contains enable_uart=1 which locks the core clock frequency
//   which is required for a stable baud rate on the auxiliary uart (uart1.)
//
// However, qemu does not implement uart1. Therefore on qemu we use uart0 for the text console.
//  Furthermore uart0 on qemu does not need any initialization.
//
pub fn initSerial(board: BootPayload) Serial {
    if (!Cpu.isQemu()) {
        // On an actual rpi, initialize uart1 to 115200 baud on pins 14 and 15:
        rpi.pinSetPullUpAndFunction(14, .None, .AlternateFunction5);
        rpi.pinSetPullUpAndFunction(15, .None, .AlternateFunction5);
        mmio.write(mmio_addr, .AUX_ENABLES, 1);
        mmio.write(mmio_addr, .AUX_MU_IER_REG, 0);
        mmio.write(mmio_addr, .AUX_MU_CNTL_REG, 0);
        mmio.write(mmio_addr, .AUX_MU_LCR_REG, 3);
        mmio.write(mmio_addr, .AUX_MU_MCR_REG, 0);
        mmio.write(mmio_addr, .AUX_MU_IER_REG, 0);
        mmio.write(mmio_addr, .AUX_MU_IIR_REG, 0xc6);
        mmio.write(mmio_addr, .AUX_MU_BAUD_REG, 270);
        mmio.write(mmio_addr, .AUX_MU_CNTL_REG, 3);
    }
    return .{
        .write = uartWriteByte,
    };
}

pub fn uartWriteByte(byte: u8) void {
    // if ascii line feed then first send carriage return
    if (byte == 10) {
        uartWriteByte(13);
    }
    if (Cpu.isQemu()) {
        // Since qemu does not implement uart1, qemu uses uart0 for the console:
        while (mmio.read(mmio_addr, .UART_FLAGS) & (1 << 5) != 0) {}
        mmio.write(mmio_addr, .UART_DATA, byte);
    } else {
        // On an actual rpi, use uart1:
        while (mmio.read(mmio_addr, .AUX_MU_LSR_REG) & (1 << 5) == 0) {}
        mmio.write(mmio_addr, .AUX_MU_IO_REG, byte);
    }
}

pub fn initMem(payload: BootPayload) std.mem.Allocator.Error!mem.MemProfile {
    log.logInfo("Init mem\n", .{});
    defer log.logInfo("Done mem\n", .{});
    mem.ADDR_OFFSET = 0;
    const phys_start = @ptrCast([*]u8, &KERNEL_PHYSADDR_START);
    const phys_end = @ptrCast([*]u8, &KERNEL_PHYSADDR_END);
    const virt_start = phys_start;
    const virt_end = phys_end;
    var allocator = std.heap.FixedBufferAllocator.init(virt_end[0..mem.FIXED_ALLOC_SIZE]);

    var allocator_region = mem.Map{ .virtual = .{ .start = @ptrToInt(virt_end), .end = @ptrToInt(virt_end) + mem.FIXED_ALLOC_SIZE }, .physical = null };
    allocator_region.physical = .{ .start = mem.virtToPhys(allocator_region.virtual.start), .end = mem.virtToPhys(allocator_region.virtual.end) };

    return mem.MemProfile{ .vaddr_end = virt_end, .vaddr_start = virt_start, .physaddr_start = phys_start, .physaddr_end = phys_end, .mem_kb = payload.memoryKB(), .modules = &[_]mem.Module{}, .virtual_reserved = &[_]mem.Map{allocator_region}, .physical_reserved = &[_]mem.Range{}, .fixed_allocator = allocator };
}

// TODO: implement
pub fn init(payload: BootPayload, mem_profile: *const mem.MemProfile, allocator: *std.mem.Allocator) void {}

// TODO: implement
pub fn inb(port: u32) u8 {
    return 0;
}

// TODO: implement
pub fn outb(port: u32, byte: u8) void {}

// TODO: implement
pub fn halt() noreturn {
    while (true) {}
}

// TODO: implement
pub fn haltNoInterrupts() noreturn {
    while (true) {}
}

// TODO: implement
pub fn spinWait() noreturn {
    while (true) {}
}

fn Res0(comptime UnsignedType: type) type {
    return enum(UnsignedType) {
        Zeroes = 0,
    };
}

fn Res1(comptime UnsignedType: type) type {
    return enum(UnsignedType) {
        Ones = @truncate(UnsignedType, 0xffffffffffffffff),
    };
}

pub const Cpu = struct {
    pub const cntfrq = systemRegisterPerExceptionLevel(0, 0, "cntfrq");
    pub const cntfrq_el0 = cntfrq.el(0);
    pub const CurrentEL = systemRegister("CurrentEL");
    pub const elr = systemRegisterPerExceptionLevel(1, 3, "elr");
    pub const elr_el1 = elr.el(1);
    pub const elr_el2 = elr.el(2);
    pub const elr_el3 = elr.el(3);
    pub const esr = systemRegisterPerExceptionLevel(1, 3, "esr");
    pub const esr_el1 = esr.el(1);
    pub const esr_el2 = esr.el(2);
    pub const esr_el3 = esr.el(3);
    pub const far = systemRegisterPerExceptionLevel(1, 3, "far");
    pub const far_el1 = far.el(1);
    pub const far_el2 = far.el(2);
    pub const far_el3 = esr.el(3);
    pub const lr = cpuRegister("lr");
    pub const mair = typedSystemRegisterPerExceptionLevel(1, 3, "mair", packed struct {
        Attr0: MemoryAttribute = .{},
        Attr1: MemoryAttribute = .{},
        Attr2: MemoryAttribute = .{},
        Attr3: MemoryAttribute = .{},
        Attr4: MemoryAttribute = .{},
        Attr5: MemoryAttribute = .{},
        Attr6: MemoryAttribute = .{},
        Attr7: MemoryAttribute = .{},
        const MemoryAttribute = packed struct {
            code: u8 = 0,
        };
    });
    pub const mair_el1 = mair.el(1);
    pub const mair_el2 = mair.el(2);
    pub const mair_el3 = mair.el(3);
    pub const midr = typedSystemRegisterPerExceptionLevel(1, 1, "midr", packed struct {
        Revison: u4,
        PartNum: u12,
        Architecture: u4,
        Variant: u4,
        Implementer: u8,
        ReservedField0: Res0(u32) = .Zeroes,
    });
    pub const mpidr = typedSystemRegisterPerExceptionLevel(1, 1, "mpidr", packed struct {
        Aff0: u8,
        Aff1: u8,
        Aff2: u8,
        MT: u1,
        ReservedField0: Res0(u5) = .Zeroes,
        U: u1,
        ReservedField1: Res1(u1) = .Ones,
        Aff3: u8,
        ReservedField2: Res0(u8) = .Zeroes,
        ReservedField3: Res0(u8) = .Zeroes,
        ReservedField4: Res0(u8) = .Zeroes,
    });
    pub const sctlr = typedSystemRegisterPerExceptionLevel(1, 3, "sctlr", packed struct {
        M: u1 = 0,
        A: u1 = 0,
        C: u1 = 0,
        SA: u1 = 0,
        ReservedField00: Res1(u2) = .Ones,
        nAA: u1 = 0,
        ReservedField01: Res0(u4) = .Zeroes,
        EOS: u1 = 0,
        I: u1 = 0,
        EnDB: u1 = 0,
        ReservedField02: Res0(u2) = .Zeroes,
        ReservedField03: Res1(u1) = .Ones,
        ReservedField04: Res0(u1) = .Zeroes,
        ReservedField05: Res1(u1) = .Ones,
        WXN: u1 = 0,
        ReservedField06: Res0(u1) = .Zeroes,
        IESB: u1 = 0,
        EIS: u1 = 0,
        ReservedField07: Res1(u1) = .Ones,
        ReservedField08: Res0(u1) = .Zeroes,
        EE: u1 = 0,
        ReservedField09: Res0(u1) = .Zeroes,
        EnDA: u1 = 0,
        ReservedField10: Res1(u2) = .Ones,
        EnlB: u1 = 0,
        EnlA: u1 = 0,
        ReservedField11: Res0(u4) = .Zeroes,
        BT: u1 = 0,
        ITFSB: u1 = 0,
        ReservedField12: Res0(u2) = .Zeroes,
        TCF: u2 = 0,
        ReservedField13: Res0(u1) = .Zeroes,
        ATA: u1 = 0,
        DSSBS: u1 = 0,
        ReservedField14: Res0(u3) = .Zeroes,
        ReservedField15: Res0(u16) = .Zeroes,
    });
    pub const sctlr_el1 = sctlr.el(1);
    pub const sctlr_el2 = sctlr.el(2);
    pub const sctlr_el3 = sctlr.el(3);
    pub const sp = cpuRegister("sp");
    pub const spsr = systemRegisterPerExceptionLevel(1, 3, "spsr");
    pub const spsr_el1 = spsr.el(1);
    pub const spsr_el2 = spsr.el(2);
    pub const spsr_el3 = spsr.el(3);
    pub const tcr = systemRegisterPerExceptionLevel(1, 3, "tcr");
    pub const tcr_el1 = tcr.el(1);
    pub const tcr_el2 = tcr.el(2);
    pub const tcr_el3 = tcr.el(3);
    pub const ttbr0 = systemRegisterPerExceptionLevel(1, 3, "ttbr0");
    pub const ttbr0_el1 = ttbr0.el(1);
    pub const ttbr0_el2 = ttbr0.el(2);
    pub const ttbr0_el3 = ttbr0.el(3);
    pub const vbar = systemRegisterPerExceptionLevel(1, 3, "vbar");
    pub const vbar_el1 = vbar.el(1);
    pub const vbar_el2 = vbar.el(2);
    pub const vbar_el3 = vbar.el(3);

    fn cpuRegister(comptime register_name: []const u8) type {
        return struct {
            pub inline fn read() usize {
                const data = asm ("mov %[data], " ++ register_name
                    : [data] "=r" (-> usize)
                );
                return data;
            }
            pub inline fn write(data: usize) void {
                asm volatile ("mov " ++ register_name ++ ", %[data]"
                    :
                    : [data] "r" (data)
                );
            }
        };
    }
    fn systemRegisterPerExceptionLevel(comptime min_level: u2, max_level: u2, comptime register_name: []const u8) type {
        return typedSystemRegisterPerExceptionLevel(min_level, max_level, register_name, usize);
    }
    fn systemRegister(comptime register_name: []const u8) type {
        return typedSystemRegister(register_name, usize);
    }
    fn typedSystemRegisterPerExceptionLevel(comptime min_level: u2, max_level: u2, comptime register_name: []const u8, comptime the_data_type: type) type {
        return struct {
            pub const data_type = the_data_type;
            pub inline fn el(exception_level: u2) type {
                const level_string = switch (exception_level) {
                    0 => "0",
                    1 => "1",
                    2 => "2",
                    3 => "3",
                };
                return typedSystemRegister(register_name ++ "_el" ++ level_string, data_type);
            }
        };
    }
    fn typedSystemRegister(comptime register_name: []const u8, comptime data_type: type) type {
        return struct {
            pub inline fn read() data_type {
                const word = asm ("mrs %[word], " ++ register_name
                    : [word] "=r" (-> usize)
                );
                return @bitCast(data_type, word);
            }
            pub inline fn readSetWrite(typed_bits: data_type) void {
                write(@bitCast(data_type, @bitCast(usize, read()) | @bitCast(usize, typed_bits)));
            }
            pub inline fn write(data: data_type) void {
                const data_usize = @bitCast(usize, data);
                asm volatile ("msr " ++ register_name ++ ", %[data_usize]"
                    :
                    : [data_usize] "r" (data_usize)
                );
            }
        };
    }
    pub fn isQemu() bool {
        return cntfrq.el(0).read() != 0;
    }
    pub inline fn isb() void {
        asm volatile (
            \\ isb
        );
    }
    pub inline fn wfe() void {
        asm volatile (
            \\ wfe
        );
    }
};

const VirtualAddress = u30;
const PhysicalAddress = u30;

const level_2_page_table_len = 2;
const level_3_page_table_len = 2 * 8 * 1024;
const table_alignment = 64 * 1024;
var translation_table: [level_3_page_table_len + level_2_page_table_len]usize align(table_alignment) = undefined;

// map 1GB to ram except last 16MB to mmio
pub fn enableFlatMmu() void {
    const page_size = 64 * 1024;
    const start_of_mmio = level_3_page_table_len - (16 * 1024 * 1024 / page_size);
    var table_index: usize = 0;
    while (table_index < start_of_mmio) : (table_index += 1) {
        translation_table[table_index] = table_index * page_size + 0x0703; // normal pte=3 attr index=0 inner shareable=3 af=1 ////////////////////////////
    }
    while (table_index < level_3_page_table_len) : (table_index += 1) {
        translation_table[table_index] = table_index * page_size + 0x0607; // device pte=3 attr index=1 outer shareable=2 af=1 ////////////////////////////
    }
    var x: usize = @ptrToInt(&translation_table);
    while (table_index < translation_table.len) : (table_index += 1) {
        translation_table[table_index] = x | 0x3; ////////////////////////////
        x += table_alignment;
    }
    Cpu.mair.el(3).write(.{
        .Attr0 = .{ .code = 0xff }, ////////////////////////////
        .Attr1 = .{ .code = 0x04 }, ////////////////////////////
    });
    const t0sz = @bitSizeOf(usize) - @bitSizeOf(VirtualAddress);
    Cpu.tcr.el(3).readSetWrite(0x80804000 | t0sz); ////////////////////////////
    Cpu.ttbr0.el(3).write(@ptrToInt(&translation_table[level_3_page_table_len]));
    Cpu.isb();
    Cpu.sctlr.el(3).readSetWrite(.{ .M = 1 });
    Cpu.isb();
}
