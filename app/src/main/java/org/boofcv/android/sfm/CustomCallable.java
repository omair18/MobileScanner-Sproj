package org.boofcv.android.sfm;
//import org.boofcv.android.sfm.DisparityCalculation;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;
import android.util.Log;

import org.ejml.data.DenseMatrix64F;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import boofcv.struct.image.GrayF32;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point3D_F64;

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
    private float sensor_width, sensor_height, focal_length;
    private DenseMatrix64F rMatrix;
    private Point3D_F64 pointRect = new Point3D_F64();
    private Point3D_F64 pointLeft = new Point3D_F64();
    private boolean isSet = false;
    private GrayF32 disparity;
    private int thread_id, numPoints =0;
    private String plycontent = "", plypoints="";
    private List<Float> vertices = new ArrayList<Float>();
    private Bitmap Srcimage;

    /* semaphores stuff */
    private final Condition condition;
    private final Semaphore[] semaphores;
    private final int index;
    private final ReentrantLock lock;
    boolean isSuspended = false;

    public CustomCallable(final Semaphore[] semaphores, final int index) {
        this.semaphores = semaphores;
        this.index = index;
        lock = new ReentrantLock();
        this.condition = lock.newCondition();
    }


    public void setter(GrayF32 disp, int rD, int minDisp, int w, int h, double bl, double f_x, double f_y, double c_x, double c_y, DenseMatrix64F r_Mat, float s_width, float s_height, float f_length, int i_th, int j_th, int t_id, boolean valuesdone, Bitmap leftimage) {

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
       sensor_width = s_width;
       sensor_height = s_height;
       focal_length = f_length;
       rMatrix = r_Mat;
       ith = i_th; // for loop iterators
       jth = j_th;
       thread_id = t_id;
       Srcimage = leftimage.copy(leftimage.getConfig(), true);
       if(valuesdone == true)
           isSet = true; //a check to see if all values have been set or not
       else
           isSet = false;
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
                float pixel_width = sensor_width/disparity.getWidth();
                float pixel_height = sensor_height/disparity.getHeight();

                for (int i = ith; i <= width; i++) {
                    for (int j = jth; j <= height; j++) {
                        double d = disparity.unsafe_get(i, j) + minDisparity;
                        int color = Srcimage.getPixel(i,j); //get color

                        /*Log.d("CALLABLE","COLOR = " + color);
                        Log.d("CALLABLE","COLOR R= " + Color.red(color));
                        Log.d("CALLABLE","COLOR G= " + Color.blue(color));
                        Log.d("CALLABLE","COLOR B= " + Color.green(color));
                        Log.d("CALLABLE","COLOR A= " + Color.alpha(color)); */

                        double depth = (disparity.unsafe_get(i, j)/255)*(rangeDispairty) + minDisparity;
                        pointRect.z = depth;
                        pointRect.x = (i*pixel_width - sensor_width/2.0)/focal_length*depth;
                        pointRect.y = (j*pixel_height - sensor_height/2.0)/focal_length*depth;

                        /*if (d >= rangeDispairty)
                            continue;
                        // coordinate in rectified camera frame
                        pointRect.z = baseline * fx / d;
                        pointRect.x = pointRect.z * (i - cx) / fx;
                        pointRect.y = pointRect.z * (j - cy) / fy; */
                        //pointRect.z =

                        //rotate into the original left camera frame
                        GeometryMath_F64.multTran(rMatrix, pointRect, pointLeft);
                        //plypoints += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + " 255 255 255\n";
                        //plypoints += Double.toString(pointLeft.x) + " " + Double.toString(pointLeft.y) + " " + Double.toString(pointLeft.z) + "\n";
                        vertices.add((float) pointLeft.x);
                        vertices.add((float) pointLeft.y);
                        vertices.add((float) pointLeft.z);
                        vertices.add((float) Color.red(color));
                        vertices.add((float) Color.blue(color));
                        vertices.add((float) Color.green(color));
                        vertices.add((float) Color.alpha(color));
                        numPoints++;
                    }
                }
                // write points to file

                //Log.d("CALLABLE", "\nPOINTSSSSSSS" + thread_id+ " == " + vertices.size());
                //DisparityCalculation.THREADS_DONE = DisparityCalculation.THREADS_DONE + 1;
                /*writeHeaders(numPoints);
                plycontent+=plypoints; // add points after headers
                String filename = "points_"+thread_id+".ply";
                appendLog(plycontent, filename, thread_id); // write file to disk */
            }

            // After work is finished, send a message to CustomThreadPoolManager
                Log.d("CALLABLE", "\n" + "\n" + "Parallel processing " + Util.MESSAGE_ID + "Thread " +
                        String.valueOf(Thread.currentThread().getId()) + " " +
                        String.valueOf(Thread.currentThread().getName()) + " completed " + thread_id + " Num Points = " + vertices.size()/3 + " " + numPoints);


            /*Message message = Util.createMessage(Util.MESSAGE_ID, "Thread " +
                    String.valueOf(Thread.currentThread().getId()) + " " +
                    String.valueOf(Thread.currentThread().getName()) + " completed " + test); */

            /// time to add all vertices into the finalvertices
            final Semaphore currentSemaphore = semaphores[index];
            final Semaphore nextSemaphore = semaphores[(index+1) %semaphores.length];

            try {
                //while (true) {
                    currentSemaphore.acquire();
                    Log.d("CALLABLE", "ITS MY TURN TO WRITE " + index);
                    lock.lock();
                    while (isSuspended) {
                        condition.await();
                    }
                    lock.unlock();
                    //sleep(300); // we use a sleep call to mock some lengthy work.
                    for(int i=0; i<vertices.size(); i++) {
                        DisparityActivity.allVerticecs.add(vertices.get(i));
                    }
                    Log.d("CALLABLE", "I'M RELEASING " + (index+1) + "CURRENT VERTICES = " + DisparityActivity.allVerticecs.size());
                    DisparityCalculation.THREADS_DONE++;
                    nextSemaphore.release();
                //}

            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            //vertices.removeAll(vertices);

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
