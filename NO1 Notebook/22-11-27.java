package src;
import javadraw.*;

// App to test the algorithm for calculating the position of the the robot
public class App extends Window {
    Rectangle robot;
    int select = 0;
    boolean up = false, left = false, right = false, down = false, turnR = false, turnL = false;
    Line sense1, sense2, sense3;
    int[] x = {-40, -40, 40};
    int[] y = {-40, 40, 40};
    int[] bx = {0, 610, 690};
    int[] by = {200, 800, 800};
    Text stats;
    String text;

    /**
     * Creates a window 800px by 800px
     * @param args no arguments are used
     */
    public static void main () {
        Window.open(800, 800, "Robot Sample");
    }

    /**
     * Main method where the program runs
     */
    public void start() {
        // Draws the robot
        robot = new Rectangle(screen, 600, 600, 100, 100);
        // Draws the lines that the sensors detect
        sense1 = new Line(screen, robot.center().x() + x[0], robot.center().y() + y[0], bx[0], by[0]);
        sense1.color(Color.RED);
        sense2 = new Line(screen, robot.center().x() + x[1], robot.center().y() + y[1], bx[1], by[1]);
        sense2.color(Color.RED);
        sense3 = new Line(screen, robot.center().x() + x[2], robot.center().y() + y[2], bx[2], by[2]);
        sense3.color(Color.RED);

        // Draws the statistics used for debugging
        stats = new Text(screen, text, 600, 20);
        
        // Main control loop
        while (true) {
            /**
             * When controling the robot
             * WASD and arrow keys move the robot
             * E and Q turn the robot clockwise and counter-clockwise
             */
            if (select == 0) {
                if (down) robot.y(robot.y()+1);
                if (up) robot.y(robot.y()-1);
                if (left) robot.x(robot.x()-1);
                if (right) robot.x(robot.x() + 1);
                if (turnL) robot.rotation(robot.rotation() - 1);
                if (turnR) robot.rotation(robot.rotation() + 1);
            /**
             * WASD and Arrow Keys move the sensors relative to the robot
             * E and Q change the angle the sensor is looking at
             */
            } else {
                if (down)  y[select - 1] += 1;
                if (up) y[select - 1] -= 1;
                if (left) x[select - 1] -= 1;
                if (right) x[select - 1] += 1;
                if (turnL)
                    if (bx[select - 1] == 0 && by[select - 1] == 0) bx[select - 1] += 1;
                    else if (bx[select - 1] == 0) by[select - 1] -= 1;
                    else if (by[select - 1] == 800) bx[select - 1] -= 1;
                    else if (bx[select - 1] == 800) by[select - 1] += 1;
                    else if (by[select - 1] == 0) bx[select - 1] += 1;
                if (turnR)
                    if (bx[select - 1] == 0 && by[select - 1] == 0) by[select - 1] += 1;
                    else if (by[select - 1] == 0) bx[select - 1] -= 1;
                    else if (bx[select - 1] == 800) by[select - 1] -= 1;
                    else if (by[select - 1] == 800) bx[select - 1] += 1;
                    else if (bx[select - 1] == 0) by[select - 1] += 1;
            }

            /** 
             * Calculates the angles and lengths based on sensor values to determine 
             * Theta, the angle the robot is facing
             * x Location of the robot
             * y Location of the robot
             */
            double hb = y[2] - y[1];
            double hd = x[2] - x[1];
            int hg = x[1];
            double de = Math.sqrt(Math.pow(sense3.pos1().x()-sense3.pos2().x(),2) + Math.pow(sense3.pos1().y()-sense3.pos2().y(),2));
            double bc = Math.sqrt(Math.pow(sense2.pos1().x()-sense2.pos2().x(),2) + Math.pow(sense2.pos1().y()-sense2.pos2().y(),2)); 
            double hde = (Math.atan((sense3.pos1().x() - sense3.pos2().x())/(sense3.pos1().y() - sense3.pos2().y()))*180/Math.PI + robot.rotation() + 90)*Math.PI/180;
            double cbp = ((- Math.atan((sense2.pos1().x() - sense2.pos2().x())/(sense2.pos1().y() - sense2.pos2().y()))*180/Math.PI - robot.rotation() + 90)*Math.PI/180);
            double hdb = Math.atan(hb/hd);
            double cbd = cbp - hdb;
            double bd = Math.sqrt(hb*hb+hd*hd);
            double bde = hde + hdb; 
            double be = Math.sqrt(bd*bd+de*de-2*bd*de*Math.cos(bde));
            double ebd = Math.acos((be*be+bd*bd-de*de)/(2*be*bd))*(bde>Math.PI?-1:1);
            double bed = Math.PI - ebd - bde;
            double cbe = cbd - ebd;
            double ce = Math.sqrt(bc*bc+be*be-2*bc*be*Math.cos(cbe));
            double bec = Math.acos((be*be+ce*ce-bc*bc)/(2*be*ce));
            double original_theta = Math.PI - hde - bed - bec;
            double anglef = (Math.atan((sense1.pos1().y() - sense1.pos2().y())/(sense1.pos1().x() - sense1.pos2().x()))*180/Math.PI - robot.rotation() + 90)*Math.PI/180;
            double distancef = Math.sqrt(Math.pow(sense1.pos1().x()-sense1.pos2().x(),2) + Math.pow(sense1.pos1().y()-sense1.pos2().y(),2));
            double closeOffset = -hg*Math.sin(original_theta) + y[1]*Math.cos(original_theta) + be*Math.sin(bec);
            double farOffset = distancef*Math.cos(anglef-original_theta-Math.PI/2) - Math.cos(original_theta)*x[0] - Math.sin(original_theta)*y[0];
        

            // Different values to display
            text = "Robot: x=" + robot.center().x() + ", y=" + (800-robot.center().y()) + 
            "\nTheta: " + robot.rotation() +
            "\nSensor 1: " + (Math.atan((sense1.pos1().y() - sense1.pos2().y())/(sense1.pos1().x() - sense1.pos2().x()))*180/Math.PI - robot.rotation()) +
            "\nSensor 2: " + (- Math.atan((sense2.pos1().x() - sense2.pos2().x())/(sense2.pos1().y() - sense2.pos2().y()))*180/Math.PI - robot.rotation() + 90) +
            "\nSensor 3: " + (Math.atan((sense3.pos1().x() - sense3.pos2().x())/(sense3.pos1().y() - sense3.pos2().y()))*180/Math.PI + robot.rotation() + 90) +
            "\nCalc x=" + (int)(farOffset+0.5) + "\nCalc y=" + (int)(closeOffset+0.5) + "\nc theta: " + ((int)(original_theta*18000000/Math.PI))/100000.0 +
            "\nclose: " + (hg*Math.sin(original_theta)) + "   " + (y[1]*Math.cos(original_theta)) + "   " + (be*Math.sin(bec)) +
            "\nfar: " + (distancef*Math.cos(anglef-original_theta-Math.PI/2))  + "    " + (- Math.cos(original_theta)*x[0]) + "    " + (- Math.sin(original_theta)*y[0]) +
            "\nde= " + de +
            "\nbc= " + bc + 
            "\nbe= " + be +
            "\nce= " + ce +
            "\nbd= " + bd +
            "\ncbe= " + (cbe*180/Math.PI) +
            "\ncbd= " + (cbd*180/Math.PI) +
            "\nebd= " + (ebd*180/Math.PI) +
            "\nbde= " + (bde*180/Math.PI) +
            "\nhde= " + (hde*180/Math.PI) + 
            "\nbed= " + (bed*180/Math.PI) +
            "\nbec= " + (bec*180/Math.PI) + 
            "\nbde= " + (bde*180/Math.PI) +
            "\nangle f: " + (anglef*180/Math.PI) +
            "\ndistancef: " + distancef +
            "\ntrue y: " + (800 - robot.center().y());

            // Redraws sensor lines
            sense1.pos1(new Location(robot.center().x() + x[0]*Math.cos(robot.rotation()*Math.PI/180) - y[0]*Math.sin(robot.rotation()*Math.PI/180), robot.center().y() + y[0]*Math.cos(robot.rotation()*Math.PI/180) + x[0]*Math.sin(robot.rotation()*Math.PI/180)));
            sense2.pos1(new Location(robot.center().x() + x[1]*Math.cos(robot.rotation()*Math.PI/180) - y[1]*Math.sin(robot.rotation()*Math.PI/180), robot.center().y() + y[1]*Math.cos(robot.rotation()*Math.PI/180) + x[1]*Math.sin(robot.rotation()*Math.PI/180)));
            sense3.pos1(new Location(robot.center().x() + x[2]*Math.cos(robot.rotation()*Math.PI/180) - y[2]*Math.sin(robot.rotation()*Math.PI/180), robot.center().y() + y[2]*Math.cos(robot.rotation()*Math.PI/180) + x[2]*Math.sin(robot.rotation()*Math.PI/180)));
            sense1.pos2(new Location(bx[0], by[0]));
            sense2.pos2(new Location(bx[1], by[1]));
            sense3.pos2(new Location(bx[2], by[2]));

            // Draws statistic text
            stats.text(text);
            
            // Updates the window display
            screen.update();
            screen.sleep(0.01);
        }
    }

    /**
     * Flags the key pressed to control robot
     * @param key the key pressed
     */
    public void keyDown(Key key) {
        if (key == Key.SPACE) select = (select+1)%4;
        if (key == Key.LEFT || key == Key.A) left = true;
        if (key == Key.RIGHT || key == Key.D) right = true;
        if (key == Key.UP || key == Key.W) up = true;
        if (key == Key.DOWN || key == Key.S) down = true;
        if (key == Key.E) turnR = true;
        if (key == Key.Q) turnL = true;
    }

    /**
     * Unflags key released to stop moving the robot
     * @param key
     */
    public void keyUp(Key key) {
        if (key == Key.LEFT || key == Key.A) left = false;
        if (key == Key.RIGHT || key == Key.D) right = false;
        if (key == Key.UP || key == Key.W) up = false;
        if (key == Key.DOWN || key == Key.S) down = false;
        if (key == Key.E) turnR = false;
        if (key == Key.Q) turnL = false;
    }
}