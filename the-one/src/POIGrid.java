import core.DTNSim;
import core.Settings;
import movement.MovementModel;

/**
 * <p>
 * Overlay grid of the world where each cell has properties related to
 * Points of Interest (POI) for the drones.
 * </p>
 * 
 * <p>
 * The idea in short:<br>
 * - Each cell in the grid represents a specific area in the world.<br>
 * - Each cell contains information about the POIs located within that area.<br>
 * - POIs can be of various types, such as targets and obstacles.<br>
 * - Each POI can have attributes like location, type and cost of passing<br>
 *   near it and the probability of being able to pass through it<br>
 *   unharmed. For example, <br>
 *     # an anti-drone area can have a cost of +16 and probability 0.5<br>
 *     # a wall can have a cost of -1(or +infinity) and probability 0 and so on.<br>
 * 
 * - The grid can be used by drones to plan their paths, taking into account<br>
 *   the costs and probabilities associated with different POIs.<br>
 * - When a drone can connect to a nearby drone or fog vehicle, it can share its
 *   knowledge of the POI grid to improve overall navigation and decision-making<br>
 *   amongst the drones.
 * </p>
 */

public class POIGrid {
    private GridCell[][] cells;
    private int cellSize;
	private int rows;
	private int cols;
	private static int worldSizeX;
	private static int worldSizeY;

    static {
		DTNSim.registerForReset(POIGrid.class.getCanonicalName());
		reset();
	}

    public static void reset() {
        Settings s = new Settings(MovementModel.MOVEMENT_MODEL_NS);
        int [] worldSize = s.getCsvInts(MovementModel.WORLD_SIZE,2);
		worldSizeX = worldSize[0];
		worldSizeY = worldSize[1];
    }

    public POIGrid() {
        this.rows = worldSizeY/cellSize + 1;
		this.cols = worldSizeX/cellSize + 1;
        this.cellSize = cellSize;

        for (int i=0; i<rows+2; i++) {
			for (int j=0; j<cols+2; j++) {
				this.cells[i][j] = new GridCell();
			}
		}
    }

    /**
	 * A single cell in the cell grid. Contains information of any POI that are
	 * currently in that part of the grid.
	 */
	public class GridCell {
        private float prob;
        private int cost;
        private GridCell() {
            this.prob = 1.0f; // Default probability of passing through is 1 (safe)
            this.cost = 0;    // Default cost is 0 (no cost)
        }

        /**
         * Get the probability of passing through this cell
         * @return probability value between 0 and 1
         */
        public float getProbability() {
            return prob;
        }

        /**
         * Get the cost associated with this cell
         * @return cost value
         */
        public int getCost() {
            return cost;
        }

        // TODO: Reset cost and probability when POIs are removed or updated??
        // TODO: A move method to move(copy?) the attributes from this cell to
        // another cell when POIs move?

        /**
         * Returns a string representation of the cell
         * @return String representation
         */
        public String toString() {
            // this.hashCode() or just this?
            return "Cost: " + cost + ", Probability: " + prob + " for cell " +
                    this;
        }

    }
}
