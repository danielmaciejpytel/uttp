// List symbols matching a regex (name or namespace) and, for vftable symbols, print the function pointer at the given slot offsets.
// Usage (postScript args): <outFile> <symbolRegex> <slotOffsetHex1> [slotOffsetHex2 ...]   e.g. "TransactionSystem" 0x420 0x428
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolIterator;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.regex.Pattern;

public class VtableSlot extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        Pattern p = Pattern.compile(args[1], Pattern.CASE_INSENSITIVE);
        Address base = currentProgram.getImageBase();
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            SymbolIterator it = currentProgram.getSymbolTable().getAllSymbols(true);
            int n = 0;
            while (it.hasNext() && !monitor.isCancelled()) {
                Symbol s = it.next();
                String full = s.getName(true);
                if (!p.matcher(full).find()) continue;
                if (n++ > 400) { out.println("... more symbols"); break; }
                out.println("SYM " + s.getAddress() + " rva=0x" + Long.toHexString(s.getAddress().subtract(base)) + " " + s.getSymbolType() + " " + full);
                if (!full.contains("vftable")) continue;
                for (int i = 2; i < args.length; i++) {
                    long off = Long.decode(args[i]);
                    Address slot = s.getAddress().add(off);
                    long ptr = currentProgram.getMemory().getLong(slot);
                    Address target = base.getNewAddress(ptr);
                    Function f = getFunctionAt(target);
                    out.println("    slot +0x" + Long.toHexString(off) + " -> " + target + " rva=0x" + Long.toHexString(target.subtract(base)) + " " + (f == null ? "<no function>" : f.getName() + " size=" + f.getBody().getNumAddresses()));
                }
            }
            out.println("symbols matched=" + n);
        }
    }
}
