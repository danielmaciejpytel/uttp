// Dump <count> qwords starting at an RVA (e.g. a vtable) with the function each one points to (if any).
// Usage (postScript args): <outFile> <rva> <count>   e.g. 0x2d88048 40
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.mem.MemoryBlock;

import java.io.FileWriter;
import java.io.PrintWriter;

public class DumpPtrs extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        long rva = Long.decode(args[1]);
        int count = Integer.parseInt(args[2]);
        Address base = currentProgram.getImageBase();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            out.println("imageBase=" + base + " start rva=0x" + Long.toHexString(rva) + " count=" + count);
            for (int i = 0; i < count; i++) {
                Address a = base.add(rva + 8L * i);
                long v = getLong(a);
                String extra = "";
                try {
                    Address t = toAddr(v);
                    MemoryBlock b = currentProgram.getMemory().getBlock(t);
                    if (b != null) {
                        Function f = getFunctionContaining(t);
                        extra = " block=" + b.getName() + (b.isExecute() ? " exec" : "")
                            + (f != null ? " func=" + f.getName() + "@rva=0x" + Long.toHexString(f.getEntryPoint().subtract(base)) : "");
                    }
                } catch (Exception e) {
                    extra = "";
                }
                out.println(String.format("+0x%03x %s val=0x%x rva=0x%x%s", 8 * i, a, v, v - base.getOffset(), extra));
            }
        }
    }
}
