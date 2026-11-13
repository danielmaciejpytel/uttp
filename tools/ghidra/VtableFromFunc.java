// Given function RVAs, find vtables containing them: raw scan of non-executable initialized blocks (.rdata/.data)
// for the little-endian function address, walk backwards while the qwords are pointers into executable memory to
// find the vtable start, then print the entries at the given slot offsets.
// Usage: <outFile> <funcRva1>[,funcRva2...] <slotOffsetHex1> [slotOffsetHex2 ...]
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.mem.MemoryBlock;

import java.io.FileWriter;
import java.io.PrintWriter;

public class VtableFromFunc extends GhidraScript {
    private Memory mem;
    private Address base;

    private boolean isCodePtr(long v) {
        try {
            MemoryBlock b = mem.getBlock(base.getNewAddress(v));
            return b != null && b.isExecute();
        } catch (Exception e) { return false; }
    }

    private long readLong(Address a) {
        try { return mem.getLong(a); } catch (Exception e) { return 0; }
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        base = currentProgram.getImageBase();
        mem = currentProgram.getMemory();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            for (String rvaS : args[1].split(",")) {
                Address fa = base.add(Long.decode(rvaS));
                Function f = getFunctionContaining(fa);
                Address entry = f == null ? fa : f.getEntryPoint();
                out.println("==== function " + rvaS + " entry " + entry + (f == null ? " <no function>" : " " + f.getName()));
                long v = entry.getOffset();
                byte[] pat = new byte[8];
                for (int i = 0; i < 8; i++) pat[i] = (byte) ((v >>> (8 * i)) & 0xff);
                for (MemoryBlock b : mem.getBlocks()) {
                    if (b.isExecute() || !b.isInitialized() || b.getName().equals(".pdata") || b.getName().equals(".reloc")) continue;
                    Address cur = b.getStart();
                    while (cur != null && cur.compareTo(b.getEnd()) < 0 && !monitor.isCancelled()) {
                        Address hit = mem.findBytes(cur, b.getEnd(), pat, null, true, monitor);
                        if (hit == null) break;
                        if ((hit.getOffset() & 7) == 0) {
                            Address start = hit;
                            int n = 0;
                            while (n < 4096) {
                                Address prev = start.subtract(8);
                                if (prev.compareTo(b.getStart()) < 0 || !isCodePtr(readLong(prev))) break;
                                start = prev; n++;
                            }
                            out.println("  hit " + hit + " rva=0x" + Long.toHexString(hit.subtract(base)) + " in " + b.getName()
                                + " -> vtable start " + start + " rva=0x" + Long.toHexString(start.subtract(base)) + " (entry at +0x" + Long.toHexString(hit.subtract(start)) + ", " + n + " code ptrs before)");
                            out.println("    qword before start: 0x" + Long.toHexString(readLong(start.subtract(8))));
                            for (int i = 2; i < args.length; i++) {
                                long off = Long.decode(args[i]);
                                long sv = readLong(start.add(off));
                                Address t = base.getNewAddress(sv);
                                Function tf = isCodePtr(sv) ? getFunctionAt(t) : null;
                                out.println("    slot +0x" + Long.toHexString(off) + " -> 0x" + Long.toHexString(sv) + " rva=0x" + Long.toHexString(sv - base.getOffset()) + " " + (tf == null ? "<no function>" : tf.getName() + " size=" + tf.getBody().getNumAddresses()));
                            }
                        }
                        cur = hit.add(1);
                    }
                }
            }
        }
    }
}
