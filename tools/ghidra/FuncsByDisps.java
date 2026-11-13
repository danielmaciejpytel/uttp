// Fingerprint search: list functions whose instructions use at least <minDistinct> of the given memory displacements
// (e.g. object field offsets "+ 0x3f6]"), with the matching instructions.
// Usage (postScript args): <outFile> <minDistinct> <maxLinesPerFunc> <dispHex1> [dispHex2 ...]   e.g. 3 40 0x3f6 0x498 0x49c
// @category UTPP
import ghidra.app.script.GhidraScript;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.FunctionIterator;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.listing.InstructionIterator;

import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class FuncsByDisps extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        int minDistinct = Integer.parseInt(args[1]);
        int maxLines = Integer.parseInt(args[2]);
        List<String> keys = new ArrayList<>();
        for (int i = 3; i < args.length; i++) keys.add("+ 0x" + Long.toHexString(Long.decode(args[i])) + "]");
        long base = currentProgram.getImageBase().getOffset();
        int hits = 0;
        try (PrintWriter out = new PrintWriter(new FileWriter(args[0]))) {
            out.println("imageBase=" + currentProgram.getImageBase() + " keys=" + keys + " minDistinct=" + minDistinct);
            FunctionIterator fit = currentProgram.getFunctionManager().getFunctions(true);
            while (fit.hasNext() && !monitor.isCancelled()) {
                Function f = fit.next();
                Set<String> seen = new LinkedHashSet<>();
                List<String> lines = new ArrayList<>();
                InstructionIterator it = currentProgram.getListing().getInstructions(f.getBody(), true);
                while (it.hasNext()) {
                    Instruction ins = it.next();
                    String s = ins.toString();
                    for (String k : keys) {
                        if (s.contains(k)) {
                            seen.add(k);
                            if (lines.size() < maxLines) lines.add("    " + ins.getAddress() + "  " + s);
                        }
                    }
                }
                if (seen.size() >= minDistinct) {
                    hits++;
                    out.println("FUNC " + f.getName() + " rva=0x" + Long.toHexString(f.getEntryPoint().getOffset() - base)
                        + " size=" + f.getBody().getNumAddresses() + " distinct=" + seen.size() + " " + seen);
                    for (String l : lines) out.println(l);
                }
            }
            out.println("functions matched=" + hits);
        }
    }
}
