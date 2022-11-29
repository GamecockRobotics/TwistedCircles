package src;
import javadraw.*;


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

    public static void main (String[] args) {
        Window.open(800, 800, "Robot Sample");
    }

    public void start() {
        robot = new Rectangle(screen, 600, 190, 100, 100);
        sense1 = new Line(screen, robot.center().x() + x[0], robot.center().y() + y[0], bx[0], by[0]);
        sense1.color(Color.RED);
        sense2 = new Line(screen, robot.center().x() + x[1], robot.center().y() + y[1], bx[1], by[1]);
        sense2.color(Color.RED);
        sense3 = new Line(screen, robot.center().x() + x[2], robot.center().y() + y[2], bx[2], by[2]);
        sense3.color(Color.RED);
        stats = new Text(screen, text, 600, 20);
        
        while (true) {

            if (select == 0) {
                if (down) robot.y(robot.y()+1);
                if (up) robot.y(robot.y()-1);
                if (left) robot.x(robot.x()-1);
                if (right) robot.x(robot.x() + 1);
                if (turnL) robot.rotation(robot.rotation() - 1);
                if (turnR) robot.rotation(robot.rotation() + 1);
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
        


            text = "Robot x:" + robot.center().x() + ", y: " + (800-robot.center().y()) + 
            "\nTheta: " + robot.rotation() +
            "\nCalc x: " + (int)(farOffset+0.5) + ", y: " + (int)(closeOffset+0.5) + 
            "\nCalculated Theta: " + ((int)(original_theta*180/Math.PI*100000))/100000.0 + 
            "\nSensor 1 Angle: " + ((int)((Math.atan((sense1.pos1().y() - sense1.pos2().y())/(sense1.pos1().x() - sense1.pos2().x()))*180/Math.PI - robot.rotation())*100000))/100000.0 +
            "\nSensor 1 Distance: " + ((int)(distancef*100000))/100000.0 +
            "\nSensor 2 Angle: " + ((int)((- Math.atan((sense2.pos1().x() - sense2.pos2().x())/(sense2.pos1().y() - sense2.pos2().y()))*180/Math.PI - robot.rotation() + 90)*100000))/100000.0 +
            "\nSensor 2 Distance: " + ((int)(bc*100000))/100000.0 + 
            "\nSensor 3 Angle: " + ((int)((Math.atan((sense3.pos1().x() - sense3.pos2().x())/(sense3.pos1().y() - sense3.pos2().y()))*180/Math.PI + robot.rotation() + 90)*100000))/100000.0 +
            "\nSensor 3 Distance: " + ((int)(de*100000))/100000.0;
            sense1.pos1(new Location(robot.center().x() + x[0]*Math.cos(robot.rotation()*Math.PI/180) - y[0]*Math.sin(robot.rotation()*Math.PI/180), robot.center().y() + y[0]*Math.cos(robot.rotation()*Math.PI/180) + x[0]*Math.sin(robot.rotation()*Math.PI/180)));
            sense2.pos1(new Location(robot.center().x() + x[1]*Math.cos(robot.rotation()*Math.PI/180) - y[1]*Math.sin(robot.rotation()*Math.PI/180), robot.center().y() + y[1]*Math.cos(robot.rotation()*Math.PI/180) + x[1]*Math.sin(robot.rotation()*Math.PI/180)));
            sense3.pos1(new Location(robot.center().x() + x[2]*Math.cos(robot.rotation()*Math.PI/180) - y[2]*Math.sin(robot.rotation()*Math.PI/180), robot.center().y() + y[2]*Math.cos(robot.rotation()*Math.PI/180) + x[2]*Math.sin(robot.rotation()*Math.PI/180)));
            sense1.pos2(new Location(bx[0], by[0]));
            sense2.pos2(new Location(bx[1], by[1]));
            sense3.pos2(new Location(bx[2], by[2]));

            stats.text(text);
            
            screen.update();
            screen.sleep(0.01);
        }
    }

    public void keyDown(Key key) {
        if (key == Key.SPACE) select = (select+1)%4;
        if (key == Key.LEFT || key == Key.A) left = true;
        if (key == Key.RIGHT || key == Key.D) right = true;
        if (key == Key.UP || key == Key.W) up = true;
        if (key == Key.DOWN || key == Key.S) down = true;
        if (key == Key.E) turnR = true;
        if (key == Key.Q) turnL = true;
    }

    public void keyUp(Key key) {
        if (key == Key.LEFT || key == Key.A) left = false;
        if (key == Key.RIGHT || key == Key.D) right = false;
        if (key == Key.UP || key == Key.W) up = false;
        if (key == Key.DOWN || key == Key.S) down = false;
        if (key == Key.E) turnR = false;
        if (key == Key.Q) turnL = false;
    }
}