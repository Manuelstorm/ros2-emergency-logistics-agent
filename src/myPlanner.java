

package fr.uga.pddl4j.AIproject;

import fr.uga.pddl4j.examples.asp.ASP;
import fr.uga.pddl4j.examples.bono.MyASP2;
import fr.uga.pddl4j.examples.bono.Node;
import fr.uga.pddl4j.heuristics.state.StateHeuristic;
import fr.uga.pddl4j.parser.DefaultParsedProblem;
import fr.uga.pddl4j.parser.RequireKey;
import fr.uga.pddl4j.plan.Plan;
import fr.uga.pddl4j.plan.SequentialPlan;
import fr.uga.pddl4j.planners.LogLevel;
import fr.uga.pddl4j.planners.AbstractPlanner;
import fr.uga.pddl4j.planners.InvalidConfigurationException;
import fr.uga.pddl4j.planners.Planner;
import fr.uga.pddl4j.planners.PlannerConfiguration;
import fr.uga.pddl4j.planners.ProblemNotSupportedException;
import fr.uga.pddl4j.problem.DefaultProblem;
import fr.uga.pddl4j.problem.Problem;
import fr.uga.pddl4j.problem.State;
import fr.uga.pddl4j.problem.operator.Action;
import fr.uga.pddl4j.problem.operator.Condition;
import fr.uga.pddl4j.problem.operator.ConditionalEffect;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import picocli.CommandLine;

import java.util.*;


@CommandLine.Command(name = "myPlanner ", version = "myPlanner 1.0", description = "Solves a specified planning problem using A* search strategy.", sortOptions = false, mixinStandardHelpOptions = true, headerHeading = "Usage:%n", synopsisHeading = "%n", descriptionHeading = "%nDescription:%n%n", parameterListHeading = "%nParameters:%n", optionListHeading = "%nOptions:%n")
public class myPlanner
    extends AbstractPlanner {

    private static final Logger LOGGER = LogManager.getLogger(myPlanner
        .class.getName());


    public static final String HEURISTIC_SETTING = "HEURISTIC";
    public static final StateHeuristic.Name DEFAULT_HEURISTIC = StateHeuristic.Name.FAST_FORWARD;
    public static final String WEIGHT_HEURISTIC_SETTING = "WEIGHT_HEURISTIC";
    public static final double DEFAULT_WEIGHT_HEURISTIC = 1.0;
    private double heuristicWeight;
    private StateHeuristic.Name heuristic;


    public myPlanner
        () {
        this(myPlanner
            .getDefaultConfiguration());
    }

    public myPlanner
    (final PlannerConfiguration configuration) {
        super();
        this.setConfiguration(configuration);
    }

    @CommandLine.Option(names = {"-w",
        "--weight"}, defaultValue = "1.0", paramLabel = "<weight>", description = "Set the weight of the heuristic (preset 1.0).")
    public void setHeuristicWeight(final double weight) {
        if (weight <= 0) {
            throw new IllegalArgumentException("Weight <= 0");
        }
        this.heuristicWeight = weight;
    }

    @CommandLine.Option(names = {"-e",
        "--heuristic"}, defaultValue = "FAST_FORWARD", description = "Set the heuristic : AJUSTED_SUM, AJUSTED_SUM2, AJUSTED_SUM2M, COMBO, "
        + "MAX, FAST_FORWARD SET_LEVEL, SUM, SUM_MUTEX (preset: FAST_FORWARD)")
    public void setHeuristic(StateHeuristic.Name heuristic) {
        this.heuristic = heuristic;
    }

    public final StateHeuristic.Name getHeuristic() {
        return this.heuristic;
    }

    public final double getHeuristicWeight() {
        return this.heuristicWeight;
    }

    @Override
    public Problem instantiate(DefaultParsedProblem problem) {
        final Problem pb = new DefaultProblem(problem);
        pb.instantiate();
        return pb;
    }

    @Override
    public Plan solve(final Problem problem) {
        LOGGER.info("* Starting My A* search \n");
        Plan plan = null;
        // Search a solution
        try {
            final long begin = System.currentTimeMillis();
            plan = this.iterative_deepening_astar_search(problem); //this.astar(problem);
            final long end = System.currentTimeMillis();
            // If a plan is found update the statistics of the planner
            // and log search information
            if (plan != null) {
                LOGGER.info("* My A* search succeeded\n");
                this.getStatistics().setTimeToSearch(end - begin);
            } else {
                LOGGER.info("* My A* search failed\n");
            }

        } catch (ProblemNotSupportedException e) {
            LOGGER.error("not supported problem");
            e.printStackTrace();
        } finally {
            // Return the plan found or null if the search fails.
            return plan;
        }
    }

    public boolean hasValidConfiguration() {
        return super.hasValidConfiguration()
            && this.getHeuristicWeight() > 0.0
            && this.getHeuristic() != null;
    }

    public static PlannerConfiguration getDefaultConfiguration() {
        PlannerConfiguration config = Planner.getDefaultConfiguration();
        config.setProperty(myPlanner.HEURISTIC_SETTING, myPlanner.DEFAULT_HEURISTIC.toString());
        config.setProperty(myPlanner.WEIGHT_HEURISTIC_SETTING,
            Double.toString(myPlanner.DEFAULT_WEIGHT_HEURISTIC));
        return config;
    }

    @Override
    public PlannerConfiguration getConfiguration() {
        final PlannerConfiguration config = super.getConfiguration();
        config.setProperty(myPlanner.HEURISTIC_SETTING, this.getHeuristic().toString());
        config.setProperty(myPlanner.WEIGHT_HEURISTIC_SETTING, Double.toString(this.getHeuristicWeight()));
        return config;
    }

    @Override
    public void setConfiguration(final PlannerConfiguration configuration) {
        super.setConfiguration(configuration);
        if (configuration.getProperty(myPlanner.WEIGHT_HEURISTIC_SETTING) == null) {
            this.setHeuristicWeight(myPlanner.DEFAULT_WEIGHT_HEURISTIC);
        } else {
            this.setHeuristicWeight(Double.parseDouble(configuration.getProperty(
                myPlanner.WEIGHT_HEURISTIC_SETTING)));
        }
        if (configuration.getProperty(myPlanner.HEURISTIC_SETTING) == null) {
            this.setHeuristic(myPlanner.DEFAULT_HEURISTIC);
        } else {
            this.setHeuristic(StateHeuristic.Name.valueOf(configuration.getProperty(
                myPlanner.HEURISTIC_SETTING)));
        }
    }


    public Plan astar(Problem problem) throws ProblemNotSupportedException {

        if (!this.isSupported(problem)) {
            throw new ProblemNotSupportedException("Problem not supported");
        }

        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);
        final State init = new State(problem.getInitialState());
        // NODI ESPLORATI
        final Set<Node> close = new HashSet<>();

        // FRONTIERA
        // f = g*w+(1-w)*h
        final double weight = this.getHeuristicWeight();
        final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                double f1 = weight * n1.getHeuristic() + (1-weight)*n1.getCost();
                double f2 = weight * n2.getHeuristic() + (1-weight)*n2.getCost();
                return Double.compare(f1, f2);
            }
        });

        //creazione nodo radice + stima dello stato iniziale
        final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));

        open.add(root);
        Plan plan = null;

        final int timeout = this.getTimeout() * 1000;
        long time = 0;
        while (!open.isEmpty() && plan == null && time < timeout) {
            // PRENDI IL PRIMO NODO
            final Node current = open.poll();
            close.add(current);

            // SE GOAL SODDISFATTO -> CALCOLO IL PIANO E LO RITORNO
            if (current.satisfy(problem.getGoal())) {
                return this.extractPlan(current, problem);
            } else {
                //SE GOAL NON SODDISFATTO DEVO APPLICARE AZIONI PER ESPANDERE IL NODO
                for (int i = 0; i < problem.getActions().size(); i++) {
                    Action a = problem.getActions().get(i);
                    //se posso applico l'azione i-esima
                    if (a.isApplicable(current)) {
                        Node next = new Node(current);
                        final List<ConditionalEffect> effects = a.getConditionalEffects(); //APPLICA L'AZIONE
                        for (ConditionalEffect ce : effects) {
                            if (current.satisfy(ce.getCondition())) {
                                next.apply(ce.getEffect());
                            }
                        }

                        // AGGIORNO IL COSTO DI 1
                        final double g = current.getCost() + 1;
                        if (!close.contains(next)) {
                            next.setCost(g);
                            next.setParent(current);
                            next.setAction(i);
                            next.setHeuristic(heuristic.estimate(next, problem.getGoal()));
                            open.add(next);
                        }
                    }
                }
            }
        }

        // RITORNA IL PIANO SE TROVATO O NULL SE NON TROVATO
        return plan;
    }


/*
    def iterative_deepening_search(problem):
        "Do depth-limited search with increasing depth limits."
        for limit in range(1, sys.maxsize):
            result = depth_limited_search(problem, limit)
        if result != cutoff:
            return result
*/
    public Plan iterative_deepening_astar_search(Problem problem) throws ProblemNotSupportedException {
        if (!this.isSupported(problem)) {
            throw new ProblemNotSupportedException("Problem not supported");
        }

        Plan result= null;
        int max_depth=Integer.MAX_VALUE;
        for(int current_depth=0; current_depth<max_depth; current_depth++){
            if(result==null)
                result=depth_limited_search(problem, current_depth);
        }
        return result;
    }

    public Plan depth_limited_search(Problem problem, int depth) {

        final StateHeuristic heuristic = StateHeuristic.getInstance(this.getHeuristic(), problem);
        final State init = new State(problem.getInitialState());
        // NODI ESPLORATI
        final Set<Node> close = new HashSet<>();

        // FRONTIERA
        // f = g*w+(1-w)*h
        final double weight = this.getHeuristicWeight();
        final PriorityQueue<Node> open = new PriorityQueue<>(100, new Comparator<Node>() {
            public int compare(Node n1, Node n2) {
                double f1 = weight * n1.getHeuristic() + n1.getCost();
                double f2 = weight * n2.getHeuristic() + n2.getCost();
                return Double.compare(f1, f2);
            }
        });
        //creazione nodo radice + stima dello stato iniziale
        final Node root = new Node(init, null, -1, 0, heuristic.estimate(init, problem.getGoal()));

        open.add(root);
        Plan plan = null;

        final int timeout = this.getTimeout() * 1000;
        long time = 0;

        while (!open.isEmpty() && plan == null && time < timeout) {
            final Node current = open.poll();
            close.add(current);

            // SE GOAL SODDISFATTO -> CALCOLO IL PIANO E LO RITORNO
            if (current.satisfy(problem.getGoal())) {
                return this.extractPlan(current, problem);
            }else if(current.getDepth()>depth){
                return null;
            } else {
                for (int i = 0; i < problem.getActions().size(); i++) {
                    Action a = problem.getActions().get(i);
                    //se posso applico l'azione i-esima
                    if (a.isApplicable(current)) {
                        Node next = new Node(current);
                        final List<ConditionalEffect> effects = a.getConditionalEffects(); //APPLICA L'AZIONE
                        for (ConditionalEffect ce : effects) {
                            if (current.satisfy(ce.getCondition())) {
                                next.apply(ce.getEffect());
                            }
                        }

                        // AGGIORNO IL COSTO DI 1
                        final double g = current.getCost() + 1;
                        if (!close.contains(next)) {
                            next.setCost(g);
                            next.setParent(current);
                            next.setAction(i);
                            next.setHeuristic(heuristic.estimate(next, problem.getGoal()));
                            open.add(next);
                        }
                    }
                }
            }

        }
        return plan;
    }

    /*
    def depth_limited_search(problem, limit=10):
        "Search deepest nodes in the search tree first."
    frontier = LIFOQueue([Node(problem.initial)])
    result = failure
    while frontier:
    node = frontier.pop()
        if problem.is_goal(node.state):
        return node
    elif len(node) >= limit:
    result = cutoff
    elif not is_cycle(node):
        for child in expand(problem, node):
        frontier.append(child)
        return result
    */

    private Plan extractPlan(final Node node, final Problem problem) {
        Node n = node;
        final Plan plan = new SequentialPlan();
        while (n.getAction() != -1) {
            final Action a = problem.getActions().get(n.getAction());
            plan.add(0, a);
            n = n.getParent();
        }
        return plan;
    }


    @Override
    public boolean isSupported(Problem problem) {
        return (problem.getRequirements().contains(RequireKey.ACTION_COSTS)
            || problem.getRequirements().contains(RequireKey.CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.CONTINOUS_EFFECTS)
            || problem.getRequirements().contains(RequireKey.DERIVED_PREDICATES)
            || problem.getRequirements().contains(RequireKey.DURATIVE_ACTIONS)
            || problem.getRequirements().contains(RequireKey.DURATION_INEQUALITIES)
            || problem.getRequirements().contains(RequireKey.FLUENTS)
            || problem.getRequirements().contains(RequireKey.GOAL_UTILITIES)
            || problem.getRequirements().contains(RequireKey.METHOD_CONSTRAINTS)
            || problem.getRequirements().contains(RequireKey.NUMERIC_FLUENTS)
            || problem.getRequirements().contains(RequireKey.OBJECT_FLUENTS)
            || problem.getRequirements().contains(RequireKey.PREFERENCES)
            || problem.getRequirements().contains(RequireKey.TIMED_INITIAL_LITERALS)
            || problem.getRequirements().contains(RequireKey.HIERARCHY))
            ? false
            : true;
    }


    public static void main(String[] args) {

        // The path to the benchmarks directory
        final String myFiles = "src/main/java/fr/uga/pddl4j/AIproject/PDDL FIles/";

        // Gets the default configuration from the planner
        fr.uga.pddl4j.planners.PlannerConfiguration config = myPlanner.getDefaultConfiguration();

        //DOMINIO E PROBLEMA
        config.setProperty(myPlanner.DOMAIN_SETTING, "C:/Users/jacop/shared/turtlebot3_ws/pddl4j/src/main/java/fr/uga/pddl4j/AIproject/PDDL FIles/logistics.pddl");
        config.setProperty(myPlanner.PROBLEM_SETTING, "C:/Users/jacop/shared/turtlebot3_ws/pddl4j/src/main/java/fr/uga/pddl4j/AIproject/PDDL FIles/instance2.pddl");

        // Dominio e problema di esempio
        //final String benchmarks = "src/test/resources/benchmarks/pddl/ipc2002/depots/strips-automatic/";
        //config.setProperty(MyASP2.DOMAIN_SETTING, benchmarks + "domain.pddl");
        //config.setProperty(MyASP2.PROBLEM_SETTING, benchmarks + "p03.pddl");

        // Sets the timeout allocated to the search.
        config.setProperty(myPlanner.TIME_OUT_SETTING, 10000);
        // Sets the log level
        config.setProperty(myPlanner.LOG_LEVEL_SETTING, LogLevel.INFO);
        // Sets the heuristic used to search
        config.setProperty(myPlanner.HEURISTIC_SETTING, StateHeuristic.Name.FAST_FORWARD);
        // Sets the weight of the heuristic
        config.setProperty(myPlanner.WEIGHT_HEURISTIC_SETTING, 1.5);

        // Creates an instance of the myPlanner planner with the specified configuration
        final myPlanner planner = new myPlanner(config);
        // Runs the planner and print the solution
        try {
            planner.solve();
        } catch (InvalidConfigurationException e) {
            e.printStackTrace();
        }
    }
}
