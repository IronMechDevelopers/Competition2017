package org.usfirst.frc.team5684.robot;

import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class ContourFinder implements Runnable {

	public ContourFinder()
	{
		
	}
	ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	@Override
	public void run() {
		/*while(!Thread.interrupted()) {
            cvSink.grabFrame(source);
            //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
            
            Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2HSV);
            
    		Core.inRange(output, new Scalar(hue[0], sat[0], val[0]),
    			new Scalar(hue[1], sat[1], val[1]), output);
            outputStream.putFrame(output);
            
            
            findContoursOutput.clear();
    		int mode;
    		if (externalOnly) {
    			mode = Imgproc.RETR_EXTERNAL;
    		}
    		else {
    			mode = Imgproc.RETR_LIST;
    		}
    		int method = Imgproc.CHAIN_APPROX_SIMPLE;
    		Imgproc.findContours(source, findContoursOutput, hierarchy, mode, method);
    		
    		for(int i=0;i<findContoursOutput.size();i++)
    		{
    			System.out.println(i+": " + findContoursOutput.get(i));
    		}
    		System.out.println("");
    		
            
        }*/
		
	}

	public ArrayList<MatOfPoint> getContours()
	{
		return findContoursOutput;
	}

}
