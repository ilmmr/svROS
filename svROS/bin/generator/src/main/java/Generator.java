
import edu.mit.csail.sdg.alloy4.A4Reporter;
import edu.mit.csail.sdg.ast.Command;
import edu.mit.csail.sdg.parser.CompModule;
import edu.mit.csail.sdg.parser.CompUtil;
import edu.mit.csail.sdg.translator.A4Options;
import edu.mit.csail.sdg.translator.A4Solution;
import edu.mit.csail.sdg.translator.A4SolutionWriter;
import edu.mit.csail.sdg.translator.TranslateAlloyToKodkod;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class Generator {
    private CompModule model;
    private A4Options options;
    private A4Reporter rep;


    public Generator(String alloy){
        rep = new A4Reporter();
        options = new A4Options();
        options.solver = A4Options.SatSolver.MiniSatProverJNI;
        model = CompUtil.parseEverything_fromFile(rep, null, alloy, 2);
    }

    public void generateRun(String type, String property){
        Command command = model.getAllCommands().stream().filter(x -> x.toString().split(" ")[1].equals(property)).collect(Collectors.toList()).get(0);
        // List<Command> commands = model.getAllCommands().stream().collect(Collectors.toList());

        /* SET DIR UP */
        File dir = new File("/tmp/generated_models/" + type);
        if (!dir.exists()) {
            dir.mkdirs();
        }

        System.out.println();
        System.out.println("Command: " + command);
        System.out.println();

        // SOLUTION
        A4Solution solution= TranslateAlloyToKodkod.execute_command(rep, model.getAllReachableSigs(), command, options);
        if (solution.satisfiable()) {
            solution.writeXML(dir + "/" + property + ".xml");
        }
    }

    public static void main(String[] args) {
        String alloy     = args[0];
        String type      = args[1];
        String property  = args[2];
        Generator g = new Generator(alloy);
        g.generateRun(type, property);
    }
}
