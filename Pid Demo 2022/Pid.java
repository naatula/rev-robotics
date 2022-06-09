package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

class ErrorRecord {
    public double e;
    public long t;
    public ErrorRecord(double e, long t) {
        this.e = e; this.t = t;
    }
}

class PIDController {
    static final int SIZE = 256;
    public double p, i, d;
    ErrorRecord[] ebuffer = new ErrorRecord[SIZE];
    int ehead = 0;
    
    public PIDController(double p, double i, double d) {
        this.p = p; this.i = i; this.d = d;
        ErrorRecord def = new ErrorRecord(0.0, 0);
        for (int idx = 0; idx < this.ebuffer.length; ++idx) {
            this.ebuffer[idx] = def;
        }
    }
    
    public void push(ErrorRecord val) {
        this.ehead += 1;
        this.ehead %= this.SIZE;
        this.ebuffer[ehead] = val;
    }
    
    public double raw_integral() {
        double sum = 0.0;
        double prevt = this.current().t;
        for (int i = 0; i < this.ebuffer.length; ++i) {
            ErrorRecord er = this.ebuffer[(this.ehead + i) % this.SIZE];
            sum += (er.t - prevt) * er.e / 1_000_000_000; // probably not correct but close enough
            prevt = er.t;
        }
        return sum;
    }
    
    public double raw_derivative() {
        ErrorRecord cur = this.current();
        ErrorRecord prev = this.ebuffer[(this.ehead - 1 + this.SIZE) % this.SIZE];
        return (cur.e - prev.e) / (cur.t - prev.t) * 1_000_000_000.0;
    }
    
    public ErrorRecord current() {
        return this.ebuffer[this.ehead];
    }
    
    public double raw_proportional() {
        return this.current().e;
    }
    
    public double proportional() {
        return Range.clip(this.p * this.raw_proportional(), -1.0, 1.0);
    }
    public double derivative() {
        return Range.clip(this.d * this.raw_derivative(), -1.0, 1.0);
    }
    public double integral() {
        return Range.clip(this.i * this.raw_integral(), -1.0, 1.0);
    }
}

public class Pid implements Runnable{
    Telemetry t;
    HardwareMap hwmap;
    bno055driver d;
    OPMode opm;
    PIDController pidc;
    
    // ultimate gain -1.8 (12.7V), oscillation period = 0.3s
    double Ku = -1.8;
    double Tu = 0.3;
    double Kp = 0.7 * Ku;
    double Ki = 0.1 * Ku / Tu;
    double Kd = 0.2 * Ku * Tu;
    
    boolean stop_signal = false;
    
    public void stop(){
        stop_signal = true;
    }
    
    public Pid(Telemetry t, HardwareMap hwmap, OPMode opm){
        this.t = t;
        this.hwmap = hwmap;
        this.opm = opm;
        this.d = new bno055driver("imu", this.hwmap);
        this.pidc = new PIDController(Kp, Ki, Kd);
    }
    
    @Override
    public void run(){
        long dt = 9999999;
        
        while(!stop_signal){
            double trueAngle = Math.toRadians(this.d.getAngles()[0]);
            double delta = this.opm.targetAngle - trueAngle;
            double min = Math.min(delta, 2 * Math.PI - delta);
            double fix =  (delta > Math.PI) ? -Math.PI * 2 : (delta < -Math.PI) ? Math.PI * 2 : 0;
            double error = (delta + fix);
            this.pidc.push(new ErrorRecord(error, System.nanoTime()));
            double modeThreshold = 0.1;
            
            double pc = this.pidc.proportional();
            double dc = this.pidc.derivative();
            double correction = pc + dc;
            // t.addData("pos",trueAngle);
            // t.addData("target",this.opm.targetAngle);
            // t.addData("correction",   pc + dc);
            // t.addData("proportional", pc);
            // t.addData("derivative",   dc);
            // t.addData("integral",     ic);
            // t.update();
            this.opm.rotate = Range.clip(pc + dc, -1, 1);
            if(Math.abs(pc + dc) < modeThreshold) {
                this.opm.rotate = 0;
            }
         }
    }
}