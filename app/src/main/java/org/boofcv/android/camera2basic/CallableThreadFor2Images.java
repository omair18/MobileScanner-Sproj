package org.boofcv.android.camera2basic;
//import org.boofcv.android.sfm.DisparityCalculation;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.os.Environment;
import android.os.Message;
import android.util.Log;
import android.widget.Toast;

import org.boofcv.android.DemoMain;
import org.boofcv.android.dmitrybrant.modelviewer.MainActivity;
import org.boofcv.android.sfm.DisparityActivity;
import org.boofcv.android.sfm.DisparityCalculation;
import org.ejml.data.DenseMatrix64F;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.Semaphore;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.disparity.StereoDisparity;
import boofcv.android.ConvertBitmap;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.feature.disparity.DisparityAlgorithms;
import boofcv.factory.feature.disparity.FactoryStereoDisparity;
import boofcv.struct.feature.BrightFeature;
import boofcv.struct.image.GrayF32;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.point.Point3D_F64;

import static boofcv.android.ConvertBitmap.grayToBitmap;

/**
 * Created by frank.yitan on 12/04/2016.
 * CustomCallable is used for sending tasks to the thread pool. When a callable is submitted,
 * a Future object is returned, allowing the thread pool manager to stop the task.
 */
public class CallableThreadFor2Images implements Callable {

    // Keep a weak reference to the CustomThreadPoolManager singleton object, so we can send a
    // message. Use of weak reference is not a must here because CustomThreadPoolManager lives
    // across the whole application lifecycle
    private WeakReference<CustomThreadPoolManager> mCustomThreadPoolManagerWeakReference;

    /* semaphores stuff */
    /*private final Condition condition;
    private final Semaphore[] semaphores;
    private final int index;
    private final ReentrantLock lock;
    boolean isSuspended = false;
    public CallableThreadFor2Images(final Semaphore[] semaphores, final int index) {
        this.semaphores = semaphores;
        this.index = index;
        lock = new ReentrantLock();
        this.condition = lock.newCondition();
    } */

    /* 2 images stuff */
    String imgpath1 = "";
    String imgpath2 = "";
    private int height = 0;
    private int width = 640;
    private boolean DONE=false;

    private DisparityCalculation<BrightFeature> disparity = null;

    public CallableThreadFor2Images(String img1path, String img2path) {
       this.imgpath1 = img1path;
        this.imgpath2 = img2path;
    }

    /* LOGIC: camera2BasicFragment will initialize this thread with 2 images and will set it to run.
              this thread will then call disparity calculations and call their threads.
     */

    @Override
    public Object call() throws Exception {
        try {
            // check if thread is interrupted before lengthy operation
            if (Thread.interrupted()) throw new InterruptedException();

            // In real world project, you might do some blocking IO operation
            // In this example, I just let the thread sleep for 3 second
            //Thread.sleep(3000);
            DisparityProcessing();
            // After work is finished, send a message to CustomThreadPoolManager
            Log.d("CallableThreadForImages", " completed and got " + this.imgpath1 + " " + this.imgpath2);
            Message message = Util.createMessage(Util.MESSAGE_ID, "CallableThreadFor2Images " +
                    String.valueOf(Thread.currentThread().getId()) + " " +
                    String.valueOf(Thread.currentThread().getName()) + " completed and got " + this.imgpath1 + " " + this.imgpath2);

            if(mCustomThreadPoolManagerWeakReference != null
                    && mCustomThreadPoolManagerWeakReference.get() != null) {

                mCustomThreadPoolManagerWeakReference.get().sendMessageToUiThread(message);
                DONE = true;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return null;
    }

    public void setCustomThreadPoolManager(CustomThreadPoolManager customThreadPoolManager) {
        this.mCustomThreadPoolManagerWeakReference = new WeakReference<CustomThreadPoolManager>(customThreadPoolManager);
    }
    private void DisparityProcessing() {
        /*BitmapFactory.Options options = new BitmapFactory.Options();
        options.inPreferredConfig = Bitmap.Config.ARGB_8888;
        options.outHeight = 288;
        options.outWidth = 352;
        Bitmap bitmap1 = BitmapFactory.decodeFile(imgpath1, options);// read both images
        Bitmap bitmap2 = BitmapFactory.decodeFile(imgpath2, options); */
        Bitmap bitmap1 = decodeFile(imgpath1); //height will be calculated using formula
        Bitmap bitmap2 = decodeFile(imgpath2);


        GrayF32 leftimage = new GrayF32(width, height);
        GrayF32 rightimage = new GrayF32(width, height);
        ConvertBitmap.bitmapToGray(bitmap2, leftimage, null);
        ConvertBitmap.bitmapToGray(bitmap1, rightimage, null);

        DetectDescribePoint<GrayF32, BrightFeature> detDesc = FactoryDetectDescribe.surfFast(null,null,null,GrayF32.class);
        ScoreAssociation<BrightFeature> score = FactoryAssociation.defaultScore(BrightFeature.class);
        AssociateDescription<BrightFeature> associate = FactoryAssociation.greedy(score,Double.MAX_VALUE,true);
        disparity = new DisparityCalculation<BrightFeature>(detDesc, associate, DemoMain.preference.intrinsic);

        long start = System.currentTimeMillis();
        disparity.init(this.width, this.height);
        disparity.setSource(leftimage);
        disparity.setDestination(rightimage);
        disparity.setDisparityAlg(createDisparity());
        disparity.rectifyImage();
        if(disparity.isDirectionLeftToRight()) {
            disparity.colorimage = bitmap1.copy(bitmap1.getConfig(),true);
        }
        else {
            disparity.colorimage = bitmap2.copy(bitmap2.getConfig(),true);
        }
        bitmap1.recycle();
        bitmap2.recycle();
        disparity.computeDisparity();
        long end = System.currentTimeMillis();

        Log.d("TWOIMAGES", "Time taken = " + Long.toString(end - start) +"miliseconds");
        while(DisparityCalculation.THREADS_DONE < 3) {
          continue;
        }
        Log.d("NAAH", "I AM GOING TO WRITE NOW " + DisparityCalculation.THREADS_DONE + " "+ DisparityActivity.allVerticecs.size() + " " + MainActivity.resultFile);

        saveGrayF32(disparity.rectifiedLeft, "r_left.png");
        saveGrayF32(disparity.rectifiedRight, "r_right.png");
        saveGrayF32(disparity.distortedLeft, "d_left.png");
        saveGrayF32(disparity.distortedRight, "d_right.png");
        saveGrayF32(disparity.getDisparity(), "disp.png");

        Log.d("TWOIMAGES", "DONE ");

        DisparityActivity.appendLog(DisparityActivity.allVerticecs, MainActivity.resultFile);

    }
    private StereoDisparity<GrayF32, GrayF32> createDisparity() {

        //return FactoryStereoDisparity.regionSubpixelWta(which,
			return FactoryStereoDisparity.regionSubpixelWta(DisparityAlgorithms.RECT_FIVE,
					1, 100, 5, 5, 30, 6, 0.1, GrayF32.class);
			//0, 100, 5, 5, 30, 6, 0.1, GrayF32.class);
		}

    private Bitmap decodeFile(String imgPath)
    {
        Bitmap b = null;
        while(b == null) {
            BitmapFactory.Options options = new BitmapFactory.Options();
            options.inPreferredConfig = Bitmap.Config.ARGB_8888;
            b = BitmapFactory.decodeFile(imgPath, options);
        }
        /* to keep aspect ratio same */
        double currentwidth = b.getWidth();
        double currentheight = b.getHeight();
        double ratio =currentwidth/currentheight;
        this.height = (int) (width / ratio);

        return Bitmap.createScaledBitmap(b, width, this.height, false);

    }
    private void saveGrayF32(GrayF32 img, String name){

        Bitmap left = grayToBitmap(img, Bitmap.Config.ARGB_8888);
        String path = Environment.getExternalStorageDirectory() + "/" + name;
							File file = new File(path);
							try {
								FileOutputStream fileOutputStream = new FileOutputStream(file);
								left.compress(Bitmap.CompressFormat.PNG, 100, fileOutputStream);
							} catch (FileNotFoundException e) {
								e.printStackTrace();
							}
    }

}
