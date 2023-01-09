package src;
import javadraw.*;

// App to test the algorithm for calculating the position of the the robot
public class App extends Window {
    Rectangle robot;
    int select = 0;
    boolean up = false, left = false, right = false, down = false, turnR = false, turnL = false;
    Line sense1, sense2, sense3;
    double[] x = {-216, -216, 216};
    double[] y = {-216, 216, 216};
    int[] t = {90, 90, 90};
    Text stats;
    String text;

    /**
     * Creates a window 800px by 800px
     * @param args no arguments are used
     */
    public static void main (String[] args) {
        Window.open(800, 800, "Robot Sample");
    }

    /**
     * Main method where the program runs
     */
    public void start() {
        // Draws the robot
        robot = new Rectangle(screen, 600, 190, 100, 100);

        // Draws the lines that the sensors detect
        sense1 = new Line(screen, robot.center().x() + x[0], robot.center().y() + y[0], 0, 0);
        sense1.color(Color.RED);
        sense2 = new Line(screen, robot.center().x() + x[1], robot.center().y() + y[1], 1, 1);
        sense2.color(Color.RED);
        sense3 = new Line(screen, robot.center().x() + x[2], robot.center().y() + y[2], 2, 2);
        sense3.color(Color.RED);

        // draws the statistics used for debugging
        stats = new Text(screen, text, 600, 20);
        

        /**
         * values for sensors to be used in debugging
         * 
         * Values from actual robot to test real situation calculations
         */ 
        double sensor1Angle    =   54       ;
        double sensor1Distance =  1000        ;
        double sensor2Angle    =     68     ;
        double sensor2Distance =     500     ;
        double sensor3Angle    =       90   ;
        double sensor3Distance =        326  ;

        // Main Control Loop
        while (true) {
            
            // Move Sensors relative to robot with arrows or WASD
            // Turn sensors with Q and E
            if (select == 0) {
                if (down) sensor1Distance -= 1;
                else if (up) sensor1Distance += 1;
                else if (left) sensor1Angle -= 1;
                else if (right) sensor1Angle += 1;
            } else if (select == 1) {
                if (down) sensor2Distance -= 1;
                else if (up) sensor2Distance += 1;
                else if (left) sensor2Angle -= 1;
                else if (right) sensor2Angle += 1;
            } else if (select == 2) {
                if (down) sensor3Distance -= 1;
                else if (up) sensor3Distance += 1;
                else if (left) sensor3Angle -= 1;
                else if (right) sensor3Angle += 1;
            }




            // Calcuklate values that can be determined by sensors or pre-match measurements
            double hb = y[2] - y[1];
            double hd = x[2] - x[1];
            double hg = x[1];
            double de = sensor3Distance;
            double bc = sensor2Distance; 
            double hde = (sensor3Angle)*Math.PI/180;
            double cbi = Math.PI - (sensor2Angle)*Math.PI/180;
            
            // Make the neccessary calculations to find the position of the robot
            double hdb = Math.atan(hb/hd);
            double cbd = cbi - hdb;
            double bd = Math.sqrt(hb*hb+hd*hd);
            double bde = hde + hdb; 
            double be = Math.sqrt(bd*bd+de*de-2*bd*de*Math.cos(bde));
            double ebd = Math.acos((be*be+bd*bd-de*de)/(2*be*bd))*(bde>Math.PI?-1:1);
            double bed = Math.PI - ebd - bde;
            double cbe = cbd - ebd;
            double ce = Math.sqrt(bc*bc+be*be-2*bc*be*Math.cos(cbe));
            double bec = Math.acos((be*be+ce*ce-bc*bc)/(2*be*ce));


            // Calculates the heading of the robot using the algorithm in the notebook
            double original_theta = Math.PI - hde - bed - bec;
            
            // Calculates the distance from the wall based on the algorithm in the notebook
            double closeOffset = be*Math.sin(bec) -hg*Math.sin(original_theta) + y[1]*Math.cos(original_theta);
            double farOffset = sensor1Distance*Math.cos((90-sensor1Angle)*Math.PI/180-original_theta) - Math.cos(original_theta)*x[0] - Math.sin(original_theta)*y[0];
        
            // Draw robot and sensors based on user input
            robot.center(farOffset/5, 800 - closeOffset/5);
            robot.rotation(-original_theta/Math.PI*180);
            sense1.pos1(new Location(robot.center().x() + x[0]*Math.cos(robot.rotation()*Math.PI/180)/5 - y[0]*Math.sin(robot.rotation()*Math.PI/180)/5, robot.center().y() + y[0]*Math.cos(robot.rotation()*Math.PI/180)/5 + x[0]*Math.sin(robot.rotation()*Math.PI/180)/5));
            sense2.pos1(new Location(robot.center().x() + x[1]*Math.cos(robot.rotation()*Math.PI/180)/5 - y[1]*Math.sin(robot.rotation()*Math.PI/180)/5, robot.center().y() + y[1]*Math.cos(robot.rotation()*Math.PI/180)/5 + x[1]*Math.sin(robot.rotation()*Math.PI/180)/5));
            sense3.pos1(new Location(robot.center().x() + x[2]*Math.cos(robot.rotation()*Math.PI/180)/5 - y[2]*Math.sin(robot.rotation()*Math.PI/180)/5, robot.center().y() + y[2]*Math.cos(robot.rotation()*Math.PI/180)/5 + x[2]*Math.sin(robot.rotation()*Math.PI/180)/5));
            sense1.pos2(calc(sense1.pos1(), (-robot.rotation()+90+sensor1Angle)*Math.PI/180));
            sense2.pos2(calc(sense2.pos1(), (-robot.rotation()+180+sensor2Angle)*Math.PI/180));
            sense3.pos2(calc(sense3.pos1(), (-robot.rotation()+180+sensor3Angle)*Math.PI/180));


            // Values for debugging
            text = 
            "x: " + (int)(farOffset+0.5) + 
            "\ny: " + (int)(closeOffset+0.5) + 
            "\nCalculated Theta: " + ((int)(original_theta*180/Math.PI*100000))/100000.0 + 
            "\nSensor 1 Angle: " + ((int)(sensor1Angle*100000))/100000.0 +
            "\nSensor 1 Distance: " + ((int)(sensor1Distance*100000))/100000.0 +
            "\nSensor 2 Angle: " + ((int)(sensor2Angle*100000))/100000.0 +
            "\nSensor 2 Distance: " + ((int)(sensor2Distance*100000))/100000.0 + 
            "\nSensor 3 Angle: " + ((int)(sensor3Angle*100000))/100000.0 +
            "\nSensor 3 Distance: " + ((int)(de*100000))/100000.0 +
            
            "\n\nreal 1: " + sense1.pos1().distance(sense1.pos2())*5 +
            "\nreal 2: " + sense2.pos1().distance(sense2.pos2())*5 +
            "\nreal 3: " + sense3.pos1().distance(sense3.pos2())*5 +

            "\n\nhb  : " + hb  + 
            "\nhd  : " + hd  + 
            "\nhg  : " + hg  + 
            "\nde  : " + de  + 
            "\nbc  : " + bc  + 
            "\nhde : " + hde + 
            "\ncbi : " + cbi + 
            "\n\nhdb : " + hdb + 
            "\ncbd : " + cbd + 
            "\nbd  : " + bd  + 
            "\nbde : " + bde + 
            "\nbe  : " + be  + 
            "\nebd : " + ebd + 
            "\nbed : " + bed + 
            "\ncbe : " + cbe + 
            "\nce  : " + ce  + 
            "\nbec : " + bec ;



            // Draws statistic text
            stats.text(text);
            
            // Updates the window display
            screen.update();
            screen.sleep(0.01);
        }
    }

    /**
     * Calculates the position on the wall that the sensor points to
     * 
     * @param loc the location of the sensor on the robot
     * @param a the distance of the sensor
     * @return the position on the wall that the sensor points to
     */
    public Location calc(Location loc, double a) {
        a= ((a%(2*Math.PI)) + 2*Math.PI)%(2*Math.PI);
        if (a < Math.atan(loc.y()/(800.0-loc.x()))) 
            return new Location(800, loc.y() - (800-loc.x())*Math.tan(a));
        else if (a < Math.PI - Math.atan(loc.y()/loc.x()))
            return new Location(loc.x() + loc.y()/Math.tan(a), 0);
        else if (a < Math.PI + Math.atan((800 - loc.y())/loc.x()))
            return new Location(0, loc.y() + loc.x()*Math.tan(a));
        else if (a < 2*Math.PI - Math.atan((800 - loc.y())/(800 - loc.x()))) 
            return new Location(loc.x() - (800 - loc.y())/Math.tan(a), 800);
        return new Location(800, loc.y() - (800-loc.x())*Math.tan(a));
    }

    /**
     * Flags the key pressed to control robot
     * @param key the key pressed
     */
    public void keyDown(Key key) {
        if (key == Key.SPACE) select = (select+1)%3;
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