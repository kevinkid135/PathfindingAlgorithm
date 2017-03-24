package Q2;

import java.util.ArrayList;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Created by Kevin on 03/24/2017.
 */
public class GridGenerator {
    public GridGenerator() {
        int xLen = ThreadLocalRandom.current().nextInt(8, 1024 + 1);
        int yLen = ThreadLocalRandom.current().nextInt(8, 1024 + 1);

        ArrayList<ArrayList<Node>> grid = new ArrayList<>();
        for (int yPos = 0; yPos < yLen; yPos++) {
            ArrayList<Node> row = new ArrayList<>();
            for (int xPos = 0; xPos < xLen; xPos++) {
                // if the edge, create wall
                if (yPos == 0 || xPos == 0 || yPos == yLen - 1 || xPos == xLen - 1) {
                    row.add(new Node('X', xPos, yPos));
                } else {
                    // randomly select wall or empty space
                    int rand = ThreadLocalRandom.current().nextInt(0, 100);
                    if (rand < 30) {
                        row.add(new Node('X', xPos, yPos));
                    } else {
                        row.add(new Node('_', xPos, yPos));
                    }
                }
            }
        }

        // select random start and goal node
        int xStart, yStart, xGoal, yGoal;
        xStart = ThreadLocalRandom.current().nextInt(1, xLen - 2);
        yStart = ThreadLocalRandom.current().nextInt(1, yLen - 2);
        do {
            xGoal = ThreadLocalRandom.current().nextInt(1, xLen - 2);
            yGoal = ThreadLocalRandom.current().nextInt(1, yLen - 2);
        } while (xStart != xGoal && yStart != yGoal);

        // set start and goal node
        grid.get(yStart).get(xStart).setType('S');
        grid.get(yGoal).get(xGoal).setType('G');
    }
}
