// ----- Processing code ---------   
Scrollbar hs1, hs2;
PImage carImg, grdImg;
int graphPos = 200, count = 0;
int[] lastGraphPos = new int[3];
int fps = 0;
float motorLimit = 1;
public PIDCtrl ctrl;
Vehicle car;
void setup() {
    size(300, 400, P2D);
    textMode(SCREEN);
    noStroke();
    hs1 = new Scrollbar(10, 190, width - 20, 10, 1);
    hs2 = new Scrollbar(0, 100, 10, 150, 1);
    carImg = loadImage("car2.png");
    grdImg = loadImage("grad.jpg");
    ctrl = new PIDCtrl(1, 0.1, 0.2);
    car = new Vehicle(.2, 0.0, radians(0));
    background(255);
}
void draw() {
    float carHeight = 50; //100 pixels = 1m   
    float carPos = hs1.getPos() / 100.0;
    float error = carPos - car.position;
    float dt = 1.0 / (frameRate + 0.000001);
    float force = ctrl.Update(error, dt);
    force = max(-motorLimit, min(motorLimit, force));
    car.slope = radians(-hs2.getPos() * .35);
    carPos = car.Update(force, dt) * 100.0;
    fill(255);
    stroke(255);
    rect(0, 0, width, 200);
    pushMatrix();
    translate(width / 2, carHeight);
    rotate(car.slope);
    pushMatrix();
    translate(-200, -7);
    scale(70, 1);
    image(grdImg, 0, 0);
    popMatrix();
    translate(carPos - 24, carHeight - 39);
    image(carImg, 0, 0);
    stroke(200, 0, 0);
    line(26, 24, 26 + ctrl.ComponentP() * 100, 24);
    stroke(0, 200, 0);
    line(26, 26, 26 + ctrl.ComponentI() * 100, 26);
    stroke(0, 0, 255);
    line(26, 28, 26 + ctrl.ComponentD() * 100, 28);
    popMatrix();
    stroke(230);
    hs1.update();
    hs2.update();
    hs1.display();
    hs2.display();
    if (count % 6 == 0)
        fps = int(frameRate);
    fill(100);
    text(fps + " fps", 5, 15);
    // graph   
    if (count % 2 == 0) {
        stroke(250);
        line(0, graphPos, 40, graphPos);
        stroke(255);
        line(41, graphPos, width, graphPos);
        int v;
        stroke(240);
        point(20, graphPos);
        v = int(force / motorLimit * 20) + 20;
        if (v == lastGraphPos[2] && (v == 0 || v == 40))
            stroke(255, 150, 150);
        else
            stroke(180, 250, 180);
        line(v, graphPos, lastGraphPos[2], graphPos);
        lastGraphPos[2] = v;
        stroke(150);
        v = int(hs1.getPos() + width / 2);
        line(v, graphPos, lastGraphPos[0], graphPos);
        lastGraphPos[0] = v;
        v = int(carPos + width / 2);
        stroke(50, 50, 255);
        line(v, graphPos, lastGraphPos[1], graphPos);
        lastGraphPos[1] = v;
        graphPos++;
        if (graphPos >= height)
            graphPos = 200;
    }
    count++;
}
void setProcessVars(float p, float i, float d, float mass, float damping, float maxf) {
    ctrl.Kp = p;
    ctrl.Ki = i;
    ctrl.Kd = d;
    car.mass = mass;
    car.damping = 1.0 - damping;
    motorLimit = maxf;
}
void resetCar() {
    ctrl.ResetCtrl();
    car.Reset();
}
class PIDCtrl {
    public float Kp, Ki, Kd;
    float lastError, integError;
    float compP, compI, compD;
    PIDCtrl(float p, float i, float d) {
        Kp = p;
        Ki = i;
        Kd = d;
        ResetCtrl();
    }
    void ResetCtrl() {
        lastError = 0.0;
        integError = 0.0;
    }
    float Update(float error, float dt) {
        integError += error * dt;
        compP = error * Kp;
        compI = integError * Ki;
        compD = ((error - lastError) / dt) * Kd;
        lastError = error;
        return compP + compI + compD;
    }
    float ComponentP() {
        return compP;
    }
    float ComponentI() {
        return compI;
    }
    float ComponentD() {
        return compD;
    }
}
class Vehicle {
    public float mass;
    public float position;
    float velocity;
    public float damping;
    public float slope;
    Vehicle(float vMass, float vDamping, float vSlope) {
        mass = vMass;
        damping = 1.0 - vDamping;
        slope = vSlope;
        Reset();
    }
    float Update(float force, float dt) {
        // consider the slope
        force += 9.81 * mass * sin(slope); // forward euler     
        float accel = force / (mass + 0.0001);
        velocity += accel * dt;
        velocity *= damping;
        position += velocity * dt;
        return position;
    }
    void Reset() {
        position = 0;
        velocity = 0;
    }
}
public class Scrollbar {
    int swidth, sheight;
    int xpos, ypos;
    float spos, newspos;
    int sposMin, sposMax;
    int sposMid;
    boolean over;
    boolean l;
    boolean locked;
    float ratio;
    public Scrollbar(int xp, int yp, int sw, int sh, int l) {
        swidth = sw;
        sheight = sh;
        ratio = (float) sw / (float) sh;
        xpos = xp;
        ypos = yp - sheight / 2;
        if (ratio >= 1.0) {
            spos = xpos + swidth / 2 - sheight / 2;
            sposMin = xpos;
            sposMax = xpos + swidth - sheight;
        } else {
            spos = ypos + sheight / 2 - swidth / 2;
            sposMin = ypos;
            sposMax = ypos + sheight - swidth;
        }
        newspos = spos;
        sposMid = (sposMax + sposMin) / 2;
    }
    public void update() {
        if (Over()) {
            over = true;
        } else {
            over = false;
        }
        if (mousePressed && over) {
            if (mouseButton == RIGHT) {
                locked = false;
                spos = sposMid;
            } else if (mouseButton == LEFT) locked = true;
        }
        if (!mousePressed) {
            locked = false;
        }
        if (locked) {
            if (ratio > 1.0) spos = constrain(mouseX - sheight / 2, sposMin, sposMax);
            else spos = constrain(mouseY - swidth / 2, sposMin, sposMax);
        }
    }
    int constrain(int val, int minv, int maxv) {
        return min(max(val, minv), maxv);
    }
    boolean Over() {
        if (mouseX > xpos && mouseX < xpos + swidth && mouseY > ypos && mouseY < ypos + sheight) {
            return true;
        } else {
            return false;
        }
    }
    void display() {
        fill(250, 160);
        rect(xpos, ypos, swidth, sheight);
        if (over || locked) {
            fill(250, 190, 0);
        } else {
            fill(200);
        }
        if (ratio > 1.0) rect(spos, ypos, sheight, sheight);
        else rect(xpos, spos, swidth, swidth);
    }
    float getPos() {
        // Convert spos to be values between     
        // 0 and the total width of the scrollbar     
        return spos - sposMid;
    }
}
// ----- end of Processing ---------