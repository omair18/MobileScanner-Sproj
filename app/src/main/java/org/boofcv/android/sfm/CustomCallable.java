package org.boofcv.android.sfm;
//import org.boofcv.android.sfm.DisparityCalculation;
import android.os.Environment;
import android.os.Message;
import android.util.Log;

import org.ejml.data.DenseMatrix64F;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.util.concurrent.Callable;

import boofcv.struct.image.GrayF32;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point3D_F64;

import org.boofcv.android.sfm.DisparityCalculation;
/**
 * Created by frank.yitan on 12/04/2016.
 * CustomCallable is used for sending tasks to the thread pool. When a callable is submitted,
 * a Future object is returned, allowing the thread pool manager to stop the task.
 */
public class CustomCallable implements Callable {

    // Keep a weak reference to the CustomThreadPoolManager singleton object, so we can send a
    // message. Use of weak reference is not a must here because CustomThreadPoolManager lives
    // across the whole application lifecycle
    private WeakReference<CustomThreadPoolManager> mCustomThreadPoolManagerWeakReference;
    private int rangeDispairty, width, height, minDisparity, ith, jth;
    private double baseline, fx, fy, cx, cy;
    private DenseMatrix64F rMatrix;
    private Point3D_F64 pointRect = new Point3D_F64();
    private Point3D_F64 pointLeft = new Point3D_F64();
    private boolean isSet = false;
    private GrayF32 disparity;
    private int thread_id, numPoints =0;
    private String plycontent = "", plypoints="";


    public void setter(GrayF32 disp, int rD, int minDisp, int w, int h, double bl, double f_x, double f_y, double c_x, double c_y, DenseMatrix64F r_Mat, int i_th, int j_th, int t_id, boolean valuesdone) {

        disparity = disp;
       rangeDispairty = rD;
       minDisparity = minDisp;
       width = w;
       height = h;
       baseline = bl;
       fx = f_x;
       fy = f_y;
       cx = c_x;
       cy = c_y;
       rMatrix = r_Mat;
       ith = i_th; // for loop iterators
       jth = j_th;
       thread_id = t_id;
       if(valuesdone == true)
           isSet = true; //a check to see if all values have been set or not
       else
           isSet = false;
    }
    public void writeHeaders(int idx) {
        plycontent += "ply\n";
        plycontent += "format ascii 1.0\n";
        plycontent+= "element vertex ";
        plycontent += Integer.toString(idx) + "\n";
        plycontent += "property float x\n";
        plycontent += "property float y\n";
        plycontent += "property float z\n";
        plycontent += "property uchar diffuse_red\n";
        plycontent += "property uchar diffuse_green\n";
        plycontent += "property uchar diffuse_blue\n";
        plycontent += "end_header\n";
    }

    public static void appendLog(String text, String fileName, int thread_id)
    {
        File logFile = new File(Environment.getExternalStorageDirectory() + "/" + fileName);
        boolean deleted = logFile.delete();
        Log.d("check123", "File details before injection: " + logFile.getAbsolutePath() + "\n" +
                "Does it exist? : " + logFile.isFile() + "\n" +
                "Is it a directory?: " +  logFile.isDirectory() + "\n" +
                "It has a length of: " + logFile.length());
        if (!logFile.exists())
        {
            Log.d("check123", "The file creation failed");
            try
            {
                logFile.createNewFile();
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        }
        try
        {
            Log.d("check123", "The file creation succeeded");
            // BufferedWriter for performance, true to set append to file flag
            BufferedWriter buf = new BufferedWriter(new FileWriter(logFile, true));
            buf.append(text);
            buf.newLine();
            buf.close();
            Log.d("check123", "File details after injection: " + logFile.getAbsolutePath() + "\n" +
                    "Does it exist? : " + logFile.isFile() + "\n" +
                    "Is it a directory?: " +  logFile.isDirectory() + "\n" +
                    "It has a length of: " + logFile.length());

            //DisparityActivity.PLYFiles_Paths[thread_id] = logFile.getAbsolutePath();
            DisparityActivity.PLYFiles_Paths[thread_id] = fileName;
        } catch (IOException e)
        {
            Log.d("check123", "IOException reached: " + e.toString());
            e.printStackTrace();
        }
    }
   @Override
    public Object call() throws Exception {
        try {
            // check if thread is interrupted before lengthy operation
            if (Thread.interrupted()) throw new InterruptedException();

            // In real world project, you might do some blocking IO operation
            // In this example, I just let the thread sleep for 3 second
            //Thread.sleep(3000);


            if (isSet == true) {
                Log.d("CALLABLE", "\n" + "\n" + "Parallel processing " + Util.MESSAGE_ID + "Thread " +
                        String.valueOf(Thread.currentThread().getId()) + " " +
                        String.valueOf(Thread.currentThread().getName()) + " STARTED " + thread_id);

                for (int i = ith; i <= width; i++) {
                    for (int j = jth; j <= height; j++) {
                        double d = disparity.unsafe_get(i, j) + minDisparity;
                        if (d >= rangeDispairty)
                            continue;
                        // coordinate in rectified camera frame
                        pointRect.z = baseline * fx / d;
                        pointRect.x = pointRect.z * (i - cx) / fx;
                        pointRect.y = pointRect.z * (j - cy) / fy;

                        //rotate into the original left camera frame
                        GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
                        //plypoints += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + " 255 255 255\n";
                        plypoints += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + "\n";
                        numPoints++;
                    }
                }
                // write points to file

                Log.d("CALLABLE", "\nPOINTSSSSSSS" + thread_id+ " == " + plypoints.length());
                //DisparityCalculation.THREADS_DONE = DisparityCalculation.THREADS_DONE + 1;
                DisparityCalculation.THREADS_DONE++;
                writeHeaders(numPoints);
                plycontent+=plypoints; // add points after headers
                String filename = "points_"+thread_id+".ply";
                appendLog(plycontent, filename, thread_id); // write file to disk
            }

            // After work is finished, send a message to CustomThreadPoolManager
                Log.d("CALLABLE", "\n" + "\n" + "Parallel processing " + Util.MESSAGE_ID + "Thread " +
                        String.valueOf(Thread.currentThread().getId()) + " " +
                        String.valueOf(Thread.currentThread().getName()) + " completed " + thread_id);


            /*Message message = Util.createMessage(Util.MESSAGE_ID, "Thread " +
                    String.valueOf(Thread.currentThread().getId()) + " " +
                    String.valueOf(Thread.currentThread().getName()) + " completed " + test); */

            if(mCustomThreadPoolManagerWeakReference != null
                    && mCustomThreadPoolManagerWeakReference.get() != null) {

                //mCustomThreadPoolManagerWeakReference.get().sendMessageToUiThread(message);
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return null;
    }

    public void setCustomThreadPoolManager(CustomThreadPoolManager customThreadPoolManager) {
        this.mCustomThreadPoolManagerWeakReference = new WeakReference<CustomThreadPoolManager>(customThreadPoolManager);
    }
}
