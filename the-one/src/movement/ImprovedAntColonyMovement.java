/*
 * Based on the improved ant colony algorithm from the paper:
 * "Adaptive multi-heuristic ant colony algorithm for UAV path planning"
 * Copyright: © 2022 Author(s). This is an open-access article distributed
 * under the terms of the Creative Commons Attribution License (CC BY 4.0)
 */
package movement;

import java.util.*;

import core.Coord;
import core.Settings;

/**
 * Movement model implementing the improved ant colony algorithm for UAV path planning.
 * The algorithm features:
 * - Adaptive heuristic function factor considering both start and destination distances
 * - Multi-heuristic factors (distance correction, safety, smoothness)
 * - Bounded pheromone values to prevent local optima
 * - Grid-based environment with 8-directional movement
 */
public class ImprovedAntColonyMovement extends MovementModel {
    /** Name space for settings */
    public static final String ANT_COLONY_NS = "ImprovedAntColony.";

    /** Grid size settings */
    public static final String GRID_WIDTH_S = "gridWidth";
    public static final String GRID_HEIGHT_S = "gridHeight";
    public static final String CELL_SIZE_S = "cellSize";

    /** Algorithm parameters */
    public static final String ALPHA_S = "alpha"; // pheromone importance factor
    public static final String BETA_S = "beta";   // heuristic importance factor
    public static final String RHO_S = "rho";     // pheromone evaporation factor
    public static final String RHO_MIN_S = "rhoMin"; // minimum evaporation factor
    public static final String Q_S = "q";         // pheromone constant
    public static final String TAU_MIN_S = "tauMin"; // minimum pheromone
    public static final String TAU_MAX_S = "tauMax"; // maximum pheromone
    public static final String TAU_INIT_S = "tauInit"; // initial pheromone
    public static final String X_COEF_S = "xCoef"; // path length coefficient
    public static final String Y_COEF_S = "yCoef"; // height variance coefficient
    public static final String Z_COEF_S = "zCoef"; // turn count coefficient

    /** Multi-heuristic parameters */
    public static final String GAMMA1_S = "gamma1"; // distance correction param 1
    public static final String G1_S = "g1";         // distance correction param 2
    public static final String GAMMA2_S = "gamma2"; // smoothness correction param 1
    public static final String G2_S = "g2";         // smoothness correction param 2
    public static final String U_S = "u";           // safety heuristic constant
    public static final String THETA_S = "theta";   // safety importance factor
    public static final String A_S = "a";           // adaptive heuristic weight A
    public static final String B_S = "b";           // adaptive heuristic weight B

    /** Obstacle configuration */
    public static final String OBSTACLE_FILE_S = "obstacleFile";

    /** Grid directions: 1-8 as per paper (1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW, 8=N) */
    private static final int[][] DIRECTION_OFFSETS = {
        {1, -1},   // 1: NE (right, up)
        {1, 0},    // 2: E (right)
        {1, 1},    // 3: SE (right, down)
        {0, 1},    // 4: S (down)
        {-1, 1},   // 5: SW (left, down)
        {-1, 0},   // 6: W (left)
        {-1, -1},  // 7: NW (left, up)
        {0, -1}    // 8: N (up)
    };

    /** Even directions (2,4,6,8) have distance 1, odd directions (1,3,5,7) have distance sqrt(2) */
    private static final int[] EVEN_DIRECTIONS = {2, 4, 6, 8};
    private static final double DIST_EVEN = 1.0;
    private static final double DIST_ODD = Math.sqrt(2);
    
    private static final int MAX_PATH_LENGTH = 1000;

    /** Grid dimensions */
    private int gridWidth;
    private int gridHeight;
    private double cellSize;

    /** Algorithm parameters */
    private double alpha;
    private double beta;
    private double rho;
    private double rhoMin;
    private double q;
    private double tauMin;
    private double tauMax;
    private double tauInit;
    private double xCoef;
    private double yCoef;
    private double zCoef;

    /** Multi-heuristic parameters */
    private double gamma1;
    private double g1;
    private double gamma2;
    private double g2;
    private double u;
    private double theta;
    private double a;
    private double b;

    /** Pheromone matrix [x][y] */
    private double[][] pheromone;

    /** Height map for terrain */
    private double[][] heightMap;

    /** Obstacle grid (true = obstacle) */
    private boolean[][] obstacles;

    /** Start and destination coordinates */
    private Coord startCoord;
    private Coord destCoord;
    private Coord initLoc;
    private int startX, startY;
    private int destX, destY;

    /** Current position in grid coordinates */
    private int currentX, currentY;
    private List<Integer> visitedPath;
    private Set<Integer> tabuList;
    private int previousDirection;

    /** Random generator */
    private Random random;

    /** Whether initialization is complete */
    private boolean initialized = false;

    /** Path to follow */
    private Path nextPath;
    private List<Coord> plannedPath;
    private int currentPathIndex;

    /**
     * Creates a new movement model based on settings
     */
    public ImprovedAntColonyMovement(Settings s) {
        super(s);
        this.random = new Random();
        this.visitedPath = new ArrayList<>();
        this.plannedPath = new ArrayList<>();
        this.currentPathIndex = 0;

        // Read grid settings
        this.gridWidth = s.getInt(ANT_COLONY_NS + GRID_WIDTH_S);
        this.gridHeight = s.getInt(ANT_COLONY_NS + GRID_HEIGHT_S);
        this.cellSize = s.getDouble(ANT_COLONY_NS + CELL_SIZE_S);

        // Read algorithm parameters
        this.alpha = s.getDouble(ANT_COLONY_NS + ALPHA_S);
        this.beta = s.getDouble(ANT_COLONY_NS + BETA_S);
        this.rho = s.getDouble(ANT_COLONY_NS + RHO_S);
        this.rhoMin = s.getDouble(ANT_COLONY_NS + RHO_MIN_S);
        this.q = s.getDouble(ANT_COLONY_NS + Q_S);
        this.tauMin = s.getDouble(ANT_COLONY_NS + TAU_MIN_S);
        this.tauMax = s.getDouble(ANT_COLONY_NS + TAU_MAX_S);
        this.tauInit = s.getDouble(ANT_COLONY_NS + TAU_INIT_S);
        this.xCoef = s.getDouble(ANT_COLONY_NS + X_COEF_S);
        this.yCoef = s.getDouble(ANT_COLONY_NS + Y_COEF_S);
        this.zCoef = s.getDouble(ANT_COLONY_NS + Z_COEF_S);

        // Read multi-heuristic parameters
        this.gamma1 = s.getDouble(ANT_COLONY_NS + GAMMA1_S);
        this.g1 = s.getDouble(ANT_COLONY_NS + G1_S);
        this.gamma2 = s.getDouble(ANT_COLONY_NS + GAMMA2_S);
        this.g2 = s.getDouble(ANT_COLONY_NS + G2_S);
        this.u = s.getDouble(ANT_COLONY_NS + U_S);
        this.theta = s.getDouble(ANT_COLONY_NS + THETA_S);
        this.a = s.getDouble(ANT_COLONY_NS + A_S);
        this.b = s.getDouble(ANT_COLONY_NS + B_S);

        // Initialize pheromone matrix
        this.pheromone = new double[gridWidth][gridHeight];
        for (int i = 0; i < gridWidth; i++) {
            for (int j = 0; j < gridHeight; j++) {
                pheromone[i][j] = tauInit;
            }
        }

        this.startCoord = new Coord(0, 0);

        // Initialize height map (simulated terrain)
        this.heightMap = new double[gridWidth][gridHeight];
        generateHeightMap();

        // Initialize obstacles
        this.obstacles = new boolean[gridWidth][gridHeight];
        initializeObstacles(s);
        this.initLoc = startCoord.clone();
    }

    /**
     * Copy constructor
     */
    public ImprovedAntColonyMovement(ImprovedAntColonyMovement proto) {
        super(proto);
        this.gridWidth = proto.gridWidth;
        this.gridHeight = proto.gridHeight;
        this.cellSize = proto.cellSize;

        this.alpha = proto.alpha;
        this.beta = proto.beta;
        this.rho = proto.rho;
        this.rhoMin = proto.rhoMin;
        this.q = proto.q;
        this.tauMin = proto.tauMin;
        this.tauMax = proto.tauMax;
        this.tauInit = proto.tauInit;
        this.xCoef = proto.xCoef;
        this.yCoef = proto.yCoef;
        this.zCoef = proto.zCoef;

        this.gamma1 = proto.gamma1;
        this.g1 = proto.g1;
        this.gamma2 = proto.gamma2;
        this.g2 = proto.g2;
        this.u = proto.u;
        this.theta = proto.theta;
        this.a = proto.a;
        this.b = proto.b;

        this.pheromone = proto.pheromone; // Share pheromone matrix among ants
        this.heightMap = proto.heightMap;
        this.obstacles = proto.obstacles;

        this.startCoord = proto.startCoord.clone();
        this.destCoord = proto.destCoord.clone();
        this.initLoc = proto.initLoc.clone();
        this.startX = proto.startX;
        this.startY = proto.startY;
        this.destX = proto.destX;
        this.destY = proto.destY;

        this.random = new Random();
        this.tabuList = new HashSet<>();
        this.visitedPath = new ArrayList<>();
        this.plannedPath = new ArrayList<>();
        this.currentPathIndex = 0;

        // Set initial position for this ant
        this.currentX = startX;
        this.currentY = startY;
        int startIndex = gridToIndex(currentX, currentY);
        this.visitedPath.add(startIndex);
        this.tabuList.add(startIndex);
        this.previousDirection = -1;

        // Find optimal path using improved ant colony algorithm
        findOptimalPath();

        this.initialized = true;
    }

    /**
     * Generate a simple height map for terrain simulation
     */
    private void generateHeightMap() {
        for (int i = 0; i < gridWidth; i++) {
            for (int j = 0; j < gridHeight; j++) {
                // Simple sinusoidal terrain
                heightMap[i][j] = 10 * Math.sin(i * 0.3) * Math.cos(j * 0.3) +
                                   5 * Math.sin(i * 0.7) * Math.cos(j * 0.7);
            }
        }
    }

    /**
     * Initialize obstacles from settings or generate random obstacles
     */
    private void initializeObstacles(Settings s) {
        // By default, no obstacles
        for (int i = 0; i < gridWidth; i++) {
            for (int j = 0; j < gridHeight; j++) {
                obstacles[i][j] = false;
            }
        }

        // Set start and destination (opposite corners by default)
        this.startX = 0;
        this.startY = 0;
        this.destX = gridWidth - 1;
        this.destY = gridHeight - 1;

        // Ensure start and dest are not obstacles
        obstacles[startX][startY] = false;
        obstacles[destX][destY] = false;

        this.startCoord = new Coord(startX * cellSize, startY * cellSize);
        this.destCoord = new Coord(destX * cellSize, destY * cellSize);

        // Add some random obstacles (20% of grid)
        int obstacleCount = (int)(gridWidth * gridHeight * 0.2);
        for (int k = 0; k < obstacleCount; k++) {
            int x = random.nextInt(gridWidth);
            int y = random.nextInt(gridHeight);
            if ((x != startX || y != startY) && (x != destX || y != destY)) {
                obstacles[x][y] = true;
            }
        }
    }

    /**
     * Convert grid coordinates to index
     */
    private int gridToIndex(int x, int y) {
        return y * gridWidth + x;
    }

    /**
     * Convert index to grid coordinates
     */
    private int[] indexToGrid(int index) {
        int[] coords = new int[2];
        coords[0] = index % gridWidth;
        coords[1] = index / gridWidth;
        return coords;
    }

    /**
     * Calculate Euclidean distance between two grid cells
     */
    private double distance(int x1, int y1, int x2, int y2) {
        double dx = (x1 - x2) * cellSize;
        double dy = (y1 - y2) * cellSize;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Check if a grid cell is valid (within bounds and not obstacle)
     */
    private boolean isValidCell(int x, int y) {
        if (x < 0 || x >= gridWidth || y < 0 || y >= gridHeight) {
            return false;
        }
        return !obstacles[x][y];
    }

    /**
     * Check if moving from current cell to adjacent cell in given direction is allowed
     * Implements the 4-step validation from paper (Section 2)
     */
    private boolean isDirectionAllowed(int currentX, int currentY, int direction) {
        int[] offset = DIRECTION_OFFSETS[direction - 1];
        int nextX = currentX + offset[0];
        int nextY = currentY + offset[1];

        // Step 1: Check for obstacles
        if (!isValidCell(nextX, nextY)) {
            return false;
        }

        // Step 3 & 4: For odd directions (diagonal), check adjacent even grids
        boolean isEven = false;
        for (int evenDir : EVEN_DIRECTIONS) {
            if (direction == evenDir) {
                isEven = true;
                break;
            }
        }

        if (!isEven) {
            // Check the two adjacent even grids for obstacles
            // For diagonal moves, both adjacent cardinal directions must be clear
            int adjDir1, adjDir2;
            switch (direction) {
                case 1: // NE
                    adjDir1 = 2; // E
                    adjDir2 = 8; // N
                    break;
                case 3: // SE
                    adjDir1 = 2; // E
                    adjDir2 = 4; // S
                    break;
                case 5: // SW
                    adjDir1 = 4; // S
                    adjDir2 = 6; // W
                    break;
                case 7: // NW
                    adjDir1 = 6; // W
                    adjDir2 = 8; // N
                    break;
                default:
                    return true;
            }

            int[] offset1 = DIRECTION_OFFSETS[adjDir1 - 1];
            int[] offset2 = DIRECTION_OFFSETS[adjDir2 - 1];

            int adjX1 = currentX + offset1[0];
            int adjY1 = currentY + offset1[1];
            int adjX2 = currentX + offset2[0];
            int adjY2 = currentY + offset2[1];

            // If either adjacent cardinal grid is obstacle, diagonal move not allowed
            if (!isValidCell(adjX1, adjY1) || !isValidCell(adjX2, adjY2)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Calculate distance from current grid to adjacent grid in given direction
     * Equation (1) from paper
     */
    private double getDis(int direction) {
        for (int evenDir : EVEN_DIRECTIONS) {
            if (direction == evenDir) {
                return DIST_EVEN;
            }
        }
        return DIST_ODD;
    }

    /**
     * Calculate distance correction function φ(i,j)
     * Equation (2) from paper
     */
    private double distanceCorrectionFunction(int x, int y, int nextX, int nextY) {
        // Find max and min distances from adjacent grids to destination
        double maxDist = 0;
        double minDist = Double.MAX_VALUE;

        for (int dir = 1; dir <= 8; dir++) {
            if (isDirectionAllowed(x, y, dir)) {
                int[] offset = DIRECTION_OFFSETS[dir - 1];
                int adjX = x + offset[0];
                int adjY = y + offset[1];
                double dist = distance(adjX, adjY, destX, destY);
                maxDist = Math.max(maxDist, dist);
                minDist = Math.min(minDist, dist);
            }
        }

        double distToDest = distance(nextX, nextY, destX, destY);
        return (maxDist - distToDest) / (maxDist - minDist + 0.001) * gamma1 * g1;
    }

    /**
     * Calculate safety function r(i,j)
     * Equation (3) from paper
     */
    private double safetyFunction(int x, int y, int nextX, int nextY, int prevDir, int currentDir) {
        if (visitedPath.isEmpty()) {
            return u / getAllowedDirections(x, y).size();
        }

        if (prevDir != -1 && prevDir == currentDir) {
            return theta * u;
        } else {
            return (1 - theta) * u / getAllowedDirections(x, y).size();
        }
    }

    /**
     * Calculate smoothness function h(i,j)
     * Equation (4) from paper
     */
    private double smoothnessFunction(int x, int y, int nextX, int nextY) {
        double maxDiff = 0;
        double minDiff = Double.MAX_VALUE;

        for (int dir = 1; dir <= 8; dir++) {
            if (isDirectionAllowed(x, y, dir)) {
                int[] offset = DIRECTION_OFFSETS[dir - 1];
                int adjX = x + offset[0];
                int adjY = y + offset[1];
                double diff = Math.abs(heightMap[x][y] - heightMap[adjX][adjY]);
                maxDiff = Math.max(maxDiff, diff);
                minDiff = Math.min(minDiff, diff);
            }
        }

        double heightDiff = Math.abs(heightMap[x][y] - heightMap[nextX][nextY]);
        return (maxDiff - heightDiff) / (maxDiff - minDiff + 0.001) * gamma2 + g2;
    }

    /**
     * Calculate adaptive heuristic function factor μ(i,j)
     * Equation (5) from paper
     */
    private double adaptiveHeuristicFactor(int x, int y) {
        double dStart = distance(startX, startY, x, y);
        double dDest = distance(x, y, destX, destY);
        return (1.0 / (a * dDest)) + (1.0 / (b * (dStart + dDest)));
    }

    /**
     * Calculate heuristic function η(i,j)
     * Equation (12) from paper
     */
    private double heuristicFunction(int x, int y, int nextX, int nextY, int direction, int prevDir) {
        double d_ij = getDis(direction);
        double phi = distanceCorrectionFunction(x, y, nextX, nextY);
        double r = safetyFunction(x, y, nextX, nextY, prevDir, direction);
        double h = smoothnessFunction(x, y, nextX, nextY);

        return (1.0 / d_ij) + phi + r + h;
    }

    /**
     * Calculate transition probability P(i,j)
     * Equation (6) from paper
     */
    private double transitionProbability(int x, int y, int nextX, int nextY,
                                          int direction, int prevDir) {
        double tau = pheromone[nextX][nextY];
        double eta = heuristicFunction(x, y, nextX, nextY, direction, prevDir);
        double mu = adaptiveHeuristicFactor(nextX, nextY);

        // Calculate denominator (sum over allowed directions)
        double sum = 0;
        List<Integer> allowed = getAllowedDirections(x, y);
        for (int dir : allowed) {
            int[] offset = DIRECTION_OFFSETS[dir - 1];
            int adjX = x + offset[0];
            int adjY = y + offset[1];
            int adjIndex = gridToIndex(adjX, adjY);
            
            if (!tabuList.contains(adjIndex)) {
                sum += Math.pow(pheromone[adjX][adjY], alpha) *
                    Math.pow(heuristicFunction(x, y, adjX, adjY, dir, prevDir), beta) *
                    adaptiveHeuristicFactor(adjX, adjY);
            }
        }

        if (sum == 0) return 0;
        return (Math.pow(tau, alpha) * Math.pow(eta, beta) * mu) / sum;
    }

    /**
     * Get all allowed directions from current position
     */
    private List<Integer> getAllowedDirections(int x, int y) {
        List<Integer> allowed = new ArrayList<>();
        for (int dir = 1; dir <= 8; dir++) {
            if (isDirectionAllowed(x, y, dir)) {
                int[] offset = DIRECTION_OFFSETS[dir - 1];
                int nextX = x + offset[0];
                int nextY = y + offset[1];

                int nextIndex = gridToIndex(nextX, nextY);

                if (!tabuList.contains(nextIndex)) {
                    allowed.add(dir);
                }
            }
        }
        return allowed;
    }

    /**
     * Select next direction based on transition probabilities
     */
    private int selectNextDirection(int x, int y, int prevDir) {
        List<Integer> allowed = getAllowedDirections(x, y);
        if (allowed.isEmpty()) {
            return -1;
        }

        // Calculate probabilities for each direction
        double[] probs = new double[allowed.size()];
        double sum = 0;

        for (int i = 0; i < allowed.size(); i++) {
            int dir = allowed.get(i);
            int[] offset = DIRECTION_OFFSETS[dir - 1];
            int nextX = x + offset[0];
            int nextY = y + offset[1];
            probs[i] = transitionProbability(x, y, nextX, nextY, dir, prevDir);
            sum += probs[i];
        }

        // Normalize probabilities
        if (sum > 0) {
            for (int i = 0; i < probs.length; i++) {
                probs[i] /= sum;
            }
        } else {
            // Equal probabilities if sum is zero
            for (int i = 0; i < probs.length; i++) {
                probs[i] = 1.0 / allowed.size();
            }
        }

        // Roulette wheel selection
        double r = random.nextDouble();
        double cumulative = 0;
        for (int i = 0; i < probs.length; i++) {
            cumulative += probs[i];
            if (r <= cumulative) {
                return allowed.get(i);
            }
        }

        return allowed.get(allowed.size() - 1);
    }

    /**
     * Find optimal path using improved ant colony algorithm
     */
    private void findOptimalPath() {
        int maxIterations = 50;
        int numAnts = 200;
        
        // Reset pheromone to initial value
        for (int i = 0; i < gridWidth; i++) {
            for (int j = 0; j < gridHeight; j++) {
                pheromone[i][j] = tauInit;
            }
        }
        
        List<List<Integer>> bestPaths = new ArrayList<>();
        List<Double> bestPathLengths = new ArrayList<>();
        
        for (int iter = 0; iter < maxIterations; iter++) {
            List<List<Integer>> antPaths = new ArrayList<>();
            List<Double> pathLengths = new ArrayList<>();
            List<Integer> pathTurns = new ArrayList<>();
            List<Double> pathHeights = new ArrayList<>();
            
            // Each ant finds a path
            for (int ant = 0; ant < numAnts; ant++) {
                List<Integer> path = new ArrayList<>();
                Set<Integer> antTabu = new HashSet<>();  // Individual ant's tabu list
                int cx = startX;
                int cy = startY;
                int prevDir = -1;
                int turnCount = 0;
                
                int startIndex = gridToIndex(cx, cy);
                path.add(startIndex);
                antTabu.add(startIndex);
                
                int steps = 0;
                boolean reachedDestination = false;
                
                while (steps < MAX_PATH_LENGTH && !reachedDestination) {
                    // Get allowed directions (excluding tabu)
                    List<Integer> allowed = new ArrayList<>();
                    for (int dir = 1; dir <= 8; dir++) {
                        if (isDirectionAllowed(cx, cy, dir)) {
                            int[] offset = DIRECTION_OFFSETS[dir - 1];
                            int nextX = cx + offset[0];
                            int nextY = cy + offset[1];
                            int nextIndex = gridToIndex(nextX, nextY);
                            
                            // Don't revisit nodes
                            if (!antTabu.contains(nextIndex)) {
                                allowed.add(dir);
                            }
                        }
                    }
                    
                    if (allowed.isEmpty()) {
                        break;  // Dead end - can't proceed
                    }
                    
                    // Calculate probabilities for allowed directions
                    double[] probs = new double[allowed.size()];
                    double sum = 0;
                    
                    for (int i = 0; i < allowed.size(); i++) {
                        int dir = allowed.get(i);
                        int[] offset = DIRECTION_OFFSETS[dir - 1];
                        int nextX = cx + offset[0];
                        int nextY = cy + offset[1];
                        
                        double tau = pheromone[nextX][nextY];
                        double eta = heuristicFunction(cx, cy, nextX, nextY, dir, prevDir);
                        double mu = adaptiveHeuristicFactor(nextX, nextY);
                        
                        probs[i] = Math.pow(tau, alpha) * Math.pow(eta, beta) * mu;
                        sum += probs[i];
                    }
                    
                    // Normalize or handle zero sum
                    if (sum > 0) {
                        for (int i = 0; i < probs.length; i++) {
                            probs[i] /= sum;
                        }
                    } else {
                        // Uniform distribution if all zero
                        for (int i = 0; i < probs.length; i++) {
                            probs[i] = 1.0 / allowed.size();
                        }
                    }
                    
                    // Select direction
                    double r = random.nextDouble();
                    double cumulative = 0;
                    int selectedDir = allowed.get(allowed.size() - 1);
                    
                    for (int i = 0; i < probs.length; i++) {
                        cumulative += probs[i];
                        if (r <= cumulative) {
                            selectedDir = allowed.get(i);
                            break;
                        }
                    }
                    
                    // Move to next cell
                    int[] offset = DIRECTION_OFFSETS[selectedDir - 1];
                    cx += offset[0];
                    cy += offset[1];
                    
                    // Check for turns
                    if (prevDir != -1 && prevDir != selectedDir) {
                        turnCount++;
                    }
                    prevDir = selectedDir;
                    
                    int currentIndex = gridToIndex(cx, cy);
                    path.add(currentIndex);
                    antTabu.add(currentIndex);  // Mark as visited
                    steps++;
                    
                    // Check if reached destination
                    if (cx == destX && cy == destY) {
                        reachedDestination = true;
                        break;
                    }
                }
                
                if (reachedDestination) {
                    antPaths.add(path);
                    
                    // Calculate path length
                    double length = calculatePathLength(path);
                    pathLengths.add(length);
                    pathTurns.add(turnCount);
                    
                    // Calculate height mean square deviation
                    double heightSum = 0;
                    for (int idx : path) {
                        int[] coords = indexToGrid(idx);
                        heightSum += heightMap[coords[0]][coords[1]];
                    }
                    double heightMean = heightSum / path.size();
                    double heightVar = 0;
                    for (int idx : path) {
                        int[] coords = indexToGrid(idx);
                        heightVar += Math.pow(heightMap[coords[0]][coords[1]] - heightMean, 2);
                    }
                    pathHeights.add(Math.sqrt(heightVar / path.size()));
                }
            }
            
            // Update pheromones (only for successful paths)
            for (int i = 0; i < antPaths.size(); i++) {
                List<Integer> path = antPaths.get(i);
                double length = pathLengths.get(i);
                double heightSD = pathHeights.get(i);
                int turns = pathTurns.get(i);
                
                // Calculate comprehensive indicator S_k(t)
                double S = xCoef * length + yCoef * heightSD + zCoef * turns;
                
                // Calculate pheromone delta
                double deltaTau = q / S;
                
                // Update pheromones along the path
                for (int idx : path) {
                    int[] coords = indexToGrid(idx);
                    pheromone[coords[0]][coords[1]] += deltaTau;
                    
                    // Apply pheromone bounds
                    if (pheromone[coords[0]][coords[1]] < tauMin) {
                        pheromone[coords[0]][coords[1]] = tauMin;
                    }
                    if (pheromone[coords[0]][coords[1]] > tauMax) {
                        pheromone[coords[0]][coords[1]] = tauMax;
                    }
                }
            }
            
            // Evaporate pheromones
            double currentRho = Math.max(rho, rhoMin);
            for (int i = 0; i < gridWidth; i++) {
                for (int j = 0; j < gridHeight; j++) {
                    pheromone[i][j] = (1 - currentRho) * pheromone[i][j];
                }
            }
            
            // Track best path
            if (!pathLengths.isEmpty()) {
                int bestIdx = 0;
                for (int i = 1; i < pathLengths.size(); i++) {
                    if (pathLengths.get(i) < pathLengths.get(bestIdx)) {
                        bestIdx = i;
                    }
                }
                bestPaths.add(antPaths.get(bestIdx));
                bestPathLengths.add(pathLengths.get(bestIdx));
            }
        }
        
        // Select overall best path and convert to coordinates
        if (!bestPathLengths.isEmpty()) {
            int overallBestIdx = 0;
            for (int i = 1; i < bestPathLengths.size(); i++) {
                if (bestPathLengths.get(i) < bestPathLengths.get(overallBestIdx)) {
                    overallBestIdx = i;
                }
            }
            
            List<Integer> bestPath = bestPaths.get(overallBestIdx);
            plannedPath.clear();
            
            // Verify path has no cycles (optional check)
            Set<Integer> pathSet = new HashSet<>();
            boolean hasCycle = false;
            for (int idx : bestPath) {
                if (pathSet.contains(idx)) {
                    hasCycle = true;
                    System.err.println("Warning: Path contains cycle at node " + idx);
                }
                pathSet.add(idx);
            }

            if (!hasCycle) {
                System.out.println("Valid path found with no revisits, length: " +
                                   bestPath.size() + " nodes");
            }
            
            for (int idx : bestPath) {
                int[] coords = indexToGrid(idx);
                plannedPath.add(new Coord(coords[0] * cellSize + cellSize/2, 
                                          coords[1] * cellSize + cellSize/2));
            }
            
            // Create path for movement
            nextPath = new Path(generateSpeed());
            for (Coord c : plannedPath) {
                nextPath.addWaypoint(c);
            }
        } else {
            System.err.println("No valid path found");
            nextPath = null;        
        }
    }

    /**
     * Calculate total length of a path
     */
    private double calculatePathLength(List<Integer> path) {
        double length = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            int[] c1 = indexToGrid(path.get(i));
            int[] c2 = indexToGrid(path.get(i + 1));
            length += distance(c1[0], c1[1], c2[0], c2[1]);
        }
        return length;
    }

    @Override
    public Coord getInitialLocation() {
        return this.initLoc;
    }

    @Override
    public Path getPath() {
        if (nextPath == null) {
            // If no path, create a new one
            findOptimalPath();
        }

        Path p = nextPath;
        nextPath = null;
        return p;
    }

    @Override
    public double nextPathAvailable() {
        if (nextPath == null) {
            return Double.MAX_VALUE;
        } else {
            return 0;
        }
    }

    @Override
    public int getMaxX() {
        return (int)(gridWidth * cellSize);
    }

    @Override
    public int getMaxY() {
        return (int)(gridHeight * cellSize);
    }

    @Override
    public ImprovedAntColonyMovement replicate() {
        return new ImprovedAntColonyMovement(this);
    }
}
