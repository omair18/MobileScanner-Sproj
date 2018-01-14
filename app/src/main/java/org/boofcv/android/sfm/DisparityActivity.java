package org.boofcv.android.sfm;
import android.app.ProgressDialog;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Bundle;
import android.view.GestureDetector;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.Toast;

import org.boofcv.android.DemoMain;
import org.boofcv.android.DemoVideoDisplayActivity;
import org.boofcv.android.R;
import org.boofcv.android.assoc.AssociationVisualize;
import org.boofcv.android.dmitrybrant.modelviewer.*;
import org.boofcv.android.dmitrybrant.modelviewer.MainActivity;

import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.disparity.StereoDisparity;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.android.ConvertBitmap;
import boofcv.android.VisualizeImageData;
import boofcv.android.gui.VideoRenderProcessing;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.feature.disparity.DisparityAlgorithms;
import boofcv.factory.feature.disparity.FactoryStereoDisparity;
import boofcv.struct.feature.BrightFeature;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageType;
import android.util.Log;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.net.Uri;
import android.os.Environment;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Semaphore;

import android.widget.ImageView;
//point cloud related imports
import boofcv.gui.d3.PointCloudViewer;
import georegression.struct.point.Point3D_F64;

/**
 * Computes the stereo disparity between two images captured by the camera.  The user selects the images and which
 * algorithm to process them using.
 *
 * @author Peter Abeles
 */
public class DisparityActivity extends DemoVideoDisplayActivity
		implements AdapterView.OnItemSelectedListener
{
	public static final int IMAGE_GALLERY_REQUEST = 20;
	private ImageView imgPicture;
	Spinner spinnerView;
	Spinner spinnerAlgs;
	public static Button ptCloudBtn;
	public static String[] PLYFiles_Paths = new String [4];
	protected static final int INPUT_BUFFER_SIZE = 0x10000;

	ProgressDialog progressDialog;
	AssociationVisualize visualize;
	// indicate where the user touched the screen
	volatile int touchEventType = 0;
	volatile int touchX;
	volatile int touchY;
	volatile boolean reset = false;

	protected float maxX;
	protected float maxY;
	protected float maxZ;
	protected float minX;
	protected float minY;
	protected float minZ;

	private GestureDetector mDetector;
	// used to notify processor that the disparity algorithms need to be changed
	int changeDisparityAlg = -1;

	public List<Float> vertices = new ArrayList<>();
	public static List<Float> allVerticecs = new ArrayList<>();

	DView activeView = DView.ASSOCIATION;

	String [] TemporaryThreadFiles = {"points_0.ply", "points_1.ply", "points_2.ply", "points_3.ply"};

	public DisparityActivity() {
		visualize = new AssociationVisualize(this);
	}

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
//		imgPicture = (ImageView) findViewById(R.id.imgSelect); when you have an image
		LayoutInflater inflater = getLayoutInflater();
		LinearLayout controls = (LinearLayout)inflater.inflate(R.layout.disparity_controls,null);

		LinearLayout parent = getViewContent();
		parent.addView(controls);

		spinnerView = (Spinner)controls.findViewById(R.id.spinner_view);
		ArrayAdapter<CharSequence> adapter = ArrayAdapter.createFromResource(this,
				R.array.disparity_views, android.R.layout.simple_spinner_item);
		adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		spinnerView.setAdapter(adapter);
		spinnerView.setOnItemSelectedListener(this);

		spinnerAlgs = (Spinner)controls.findViewById(R.id.spinner_algs);
		adapter = ArrayAdapter.createFromResource(this,
				R.array.disparity_algs, android.R.layout.simple_spinner_item);
		adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		spinnerAlgs.setAdapter(adapter);
		spinnerAlgs.setOnItemSelectedListener(this);

		ptCloudBtn = (Button)findViewById(R.id.button_ptcloud);

		//ptCloudBtn.setVisibility(View.INVISIBLE);
		//DeleteAllFiles();
		FrameLayout iv = getViewPreview();
		mDetector = new GestureDetector(this, new MyGestureDetector(iv));
		iv.setOnTouchListener(new View.OnTouchListener(){
			@Override
			public boolean onTouch(View v, MotionEvent event)
			{
				mDetector.onTouchEvent(event);
				return true;
			}});
	}

	@Override
	protected void onResume() {
		super.onResume();
		//DeleteAllFiles();
		setProcessing(new DisparityProcessing());
		visualize.setSource(null);
		visualize.setDestination(null);
		changeDisparityAlg = spinnerAlgs.getSelectedItemPosition();
	}

	@Override
	public void onItemSelected(AdapterView<?> adapterView, View view, int pos, long id ) {
		if( adapterView == spinnerView ) {
			if( pos == 0 ) {
				activeView = DView.ASSOCIATION;
			} else if( pos == 1 ) {
				touchY = -1;
				activeView = DView.RECTIFICATION;
			} else {
				activeView = DView.DISPARITY;
			}
		} else if( adapterView == spinnerAlgs ) {
			changeDisparityAlg = pos;
		}
	}

	@Override
	public void onNothingSelected(AdapterView<?> adapterView) {}

	protected void adjustMaxMin(float x, float y, float z) {
		if (x > maxX) {
			maxX = x;
		}
		if (y > maxY) {
			maxY = y;
		}
		if (z > maxZ) {
			maxZ = z;
		}
		if (x < minX) {
			minX = x;
		}
		if (y < minY) {
			minY = y;
		}
		if (z < minZ) {
			minZ = z;
		}
	}

	public void readPLYText(List<Float> vertices, String fileName) throws IOException {
		String path = Environment.getExternalStorageDirectory() + "/" + fileName;
		File file = new File(path);
		FileInputStream fileInputStream = new FileInputStream(file);
		BufferedInputStream stream = new BufferedInputStream(fileInputStream, INPUT_BUFFER_SIZE);
		BufferedReader reader = new BufferedReader(new InputStreamReader(stream), INPUT_BUFFER_SIZE);

		String line;
		String[] lineArr;
		int vertexCount = 0;

		stream.mark(0x100000);
		boolean isBinary = false;
		while ((line = reader.readLine()) != null) {
			line = line.trim();
			if (line.startsWith("format ")) {
				if (line.contains("binary")) {
					isBinary = true;
				}
			} else if (line.startsWith("element vertex")) {
				lineArr = line.split(" ");
				vertexCount = Integer.parseInt(lineArr[2]);
			} else if (line.startsWith("end_header")) {
				break;
			}
		} //

		float x, y, z;

		double centerMassX = 0.0;
		double centerMassY = 0.0;
		double centerMassZ = 0.0;

		for (int i = 0; i < vertexCount; i++) {
			lineArr = reader.readLine().trim().split(" ");
			x = Float.parseFloat(lineArr[0]);
			y = Float.parseFloat(lineArr[1]);
			z = Float.parseFloat(lineArr[2]);
			vertices.add(x);
			vertices.add(y);
			vertices.add(z);
			adjustMaxMin(x,y,z);
		}
	}

	private Boolean checkIfAllFilesExist() {

		File p0 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[0]);
		File p1 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[1]);
		File p2 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[2]);
		File p3 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[3]);

		if(p0.isFile() && p1.isFile() && p2.isFile() && p3.isFile())
			return true;
		else
			return false;
	}
	private void DeleteAllFiles() {

		File p0 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[0]);
		File p1 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[1]);
		File p2 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[2]);
		File p3 = new File(Environment.getExternalStorageDirectory() + "/" + TemporaryThreadFiles[3]);

		p0.delete();
		p1.delete();
		p2.delete();
		p3.delete();

	}

public static void appendLog(List<Float> vertices, String fileName)
    {
        File logFile = new File(Environment.getExternalStorageDirectory() + "/" + fileName);
        boolean deleted = logFile.delete();
        Log.d("checkFINALFILE", "File details before injection: " + logFile.getAbsolutePath() + "\n" +
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
			buf.append("ply\n");
        	buf.append("format ascii 1.0\n");
        	buf.append("element vertex ");
        	int num = vertices.size()/3;
        	buf.append(Integer.toString(num)+ "\n");
        	buf.append("property float x\n");
        	buf.append("property float y\n");
        	buf.append("property float z\n");
			buf.append("end_header\n");
            for(int i=0; i<vertices.size(); i++) {
				buf.append(Float.toString(vertices.get(i)));
				buf.append(" ");
				buf.append(Float.toString(vertices.get(++i)));
				buf.append(" ");
				buf.append(Float.toString(vertices.get(++i)));
				buf.newLine();
			}
            buf.close();
            Log.d("checkFINALFILE", "File details after injection: " + logFile.getAbsolutePath() + "\n" +
                    "Does it exist? : " + logFile.isFile() + "\n" +
                    "Is it a directory?: " +  logFile.isDirectory() + "\n" +
                    "It has a length of: " + logFile.length());

            //DisparityActivity.PLYFiles_Paths[thread_id] = logFile.getAbsolutePath();
        } catch (IOException e)
        {
            Log.d("checkFINALFILE", "IOException reached: " + e.toString());
            e.printStackTrace();
        }
    }

	public void onClickPointCloudButton(View v) {

		/*File logFile = new File(Environment.getExternalStorageDirectory() + "/" + MainActivity.resultFile);
        boolean deleted = logFile.delete(); */
		progressDialog = new ProgressDialog(DisparityActivity.this);
		progressDialog.setMessage("Loading..."); // Setting Message
		progressDialog.setTitle("Computing PointCloud"); // Setting Title
		progressDialog.setProgressStyle(ProgressDialog.STYLE_SPINNER); // Progress Dialog Style Spinner
		progressDialog.show(); // Display Progress Dialog
		progressDialog.setCancelable(true);
		new Thread(new Runnable() {
			public void run() {
				try {
					while(DisparityCalculation.THREADS_DONE < 3) {
						Toast.makeText(DisparityActivity.this, "WAITING FOR THREADS TO FINISH THEIR TASK!", Toast.LENGTH_SHORT).show();
						/*try {
							Thread.sleep(500);
						} catch (InterruptedException e) {
							e.printStackTrace();
						} */
					}
					/*while(checkIfAllFilesExist() != true) { //check if all files have been written to the disk
						Thread.sleep(500);
					} */
					//read all plyfiles
					/*for(int j=0; j < PLYFiles_Paths.length; j++) {
						try {
							readPLYText(vertices, PLYFiles_Paths[j]);
						} catch (IOException e) {
							e.printStackTrace();
						}
					} */
					/*for(int k=0; k<vertices.size(); k++) {
						MainActivity.vertices.add(vertices.get(k));
					} */
					appendLog(allVerticecs,MainActivity.resultFile); //write vertices into the resultfile

					//Log.v("VERTICES", "Length of vertices = "+ vertices.size()+ " " + MainActivity.vertices.size());
					Log.v("F_VERTICES", "Length of final vertices = "+ allVerticecs.size()/3);

					Intent i = new Intent(DisparityActivity.this, org.boofcv.android.dmitrybrant.modelviewer.MainActivity.class); //start pointcloud activity
					startActivity(i); //start acvitity
					//Thread.sleep(10000);
				} catch (Exception e) {
					e.printStackTrace();
				}
				progressDialog.dismiss();

			}
		}).start();
	}

	public void resetPressed( View view ) {
        reset = true;
        File logFile = new File(Environment.getExternalStorageDirectory() + "/" + MainActivity.resultFile);
        boolean deleted = logFile.delete();

        try {
            java.io.File xmlFile = new java.io.File(Environment
                    .getDataDirectory()
                    + "/Filename.txt");
            String data = "checking" + "\n" + "Hi there";
            org.apache.commons.io.FileUtils.writeStringToFile(xmlFile, data, "UTF-8");
        } catch(Exception e) {
            Log.d("pcloud", "In activity: " + e.toString());
        }
    }

	protected class MyGestureDetector extends GestureDetector.SimpleOnGestureListener
	{
		View v;

		public MyGestureDetector(View v) {
			this.v = v;
		}

		@Override
		public boolean onDown(MotionEvent e) {

			// make sure the camera is calibrated first
			if( DemoMain.preference.intrinsic == null ) {
				Toast.makeText(DisparityActivity.this, "You must first calibrate the camera!", Toast.LENGTH_SHORT).show();
				return false;
			}

			if( activeView == DView.ASSOCIATION ) {
				touchEventType = 1;
				touchX = (int)e.getX();
				touchY = (int)e.getY();
			} else if( activeView == DView.RECTIFICATION ) {
				touchY = (int)e.getY();
			}

			return true;
		}

		/**
		 * If the user flings an image discard the results in the image
		 */
		@Override
		public boolean onFling( MotionEvent e1, MotionEvent e2, float velocityX, float velocityY) {
			if( activeView != DView.ASSOCIATION ) {
				return false;
			}

			touchEventType = (int)e1.getX() < v.getWidth()/2 ? 2 : 3;

			return true;
		}

		@Override
		public boolean onDoubleTapEvent(MotionEvent e)
		{
			if( activeView != DView.ASSOCIATION ) {
				return false;
			}

			touchEventType = 4;
			return true;
		}
	}

//This is where the point cloud processing generation begins. Fnu Tulha//
	protected class DisparityProcessing extends VideoRenderProcessing<GrayF32> {

		DisparityCalculation<BrightFeature> disparity;

		GrayF32 disparityImage;
		int disparityMin,disparityMax;
//PointCloudViewer viewer = new PointCloudViewer(rectK, 10);

		public DisparityProcessing() {
			super(ImageType.single(GrayF32.class));

			DetectDescribePoint<GrayF32, BrightFeature> detDesc =
					FactoryDetectDescribe.surfFast(null,null,null,GrayF32.class);

			ScoreAssociation<BrightFeature> score = FactoryAssociation.defaultScore(BrightFeature.class);
			AssociateDescription<BrightFeature> associate =
					FactoryAssociation.greedy(score,Double.MAX_VALUE,true);

			disparity = new DisparityCalculation<BrightFeature>(detDesc,associate,DemoMain.preference.intrinsic);
		}

		@Override
		protected void declareImages(int width, int height) {
			super.declareImages(width, height);

			disparityImage = new GrayF32(width,height);

			visualize.initializeImages( width, height );
			outputWidth = visualize.getOutputWidth();
			outputHeight = visualize.getOutputHeight();

			disparity.init(width,height);
		}

		private StereoDisparity<GrayF32, GrayF32> createDisparity() {

			DisparityAlgorithms which;
			switch( changeDisparityAlg ) {
				case 0:
					which = DisparityAlgorithms.RECT;
					break;

				case 1:
					which = DisparityAlgorithms.RECT_FIVE;
					break;

				default:
					throw new RuntimeException("Unknown algorithm " + changeDisparityAlg);
			}


			return FactoryStereoDisparity.regionSubpixelWta(which,
					5, 40, 5, 5, 100, 1, 0.1, GrayF32.class);
		}

		@Override
		protected void process(GrayF32 gray) {
            Context cx;
			int target = 0;
			// process GUI interactions
			synchronized ( lockGui ) {
				if( reset ) {
					reset = false;
					visualize.setSource(null);
					visualize.setDestination(null);
					runOnUiThread(new Runnable() {
						@Override
						public void run() {
							spinnerView.setSelection(0);
						}
					});

				}
				if( touchEventType == 1 ) {
					// first see if there are any features to select
					if( !visualize.setTouch(touchX,touchY) ) {
						// if not then it must be a capture image request
						target = touchX < view.getWidth()/2 ? 1 : 2;
					}
				} else if( touchEventType == 2 ) {
					visualize.setSource(null);
					visualize.forgetSelection();
				} else if( touchEventType == 3 ) {
					visualize.setDestination(null);
					visualize.forgetSelection();
				} else if( touchEventType == 4 ) {
					visualize.forgetSelection();
				}
			}
			touchEventType = 0;

			boolean computedFeatures = false;
			// compute image features for left or right depending on user selection
			if( target == 1 ) {
				long start = System.currentTimeMillis( );
				setProgressMessage("Detecting Features Left");
				disparity.setSource(gray);
				long end = System.currentTimeMillis( );
				long diff = end - start;
				Log.d("rect", Long.toString(diff) + " " + "left features");
				computedFeatures = true;
			} else if( target == 2 ) {
				long start = System.currentTimeMillis( );
				setProgressMessage("Detecting Features Right");
				disparity.setDestination(gray);
				long end = System.currentTimeMillis( );
				long diff = end - start;
				Log.d("rect", Long.toString(diff) + " " + "right features");
				computedFeatures = true;
			}

			synchronized ( lockGui ) {
				if( target == 1 ) {
					visualize.setSource(gray);
				} else if( target == 2 ) {
					visualize.setDestination(gray);
				}
			}

			if( changeDisparityAlg != -1 ) {
				disparity.setDisparityAlg(createDisparity());
			}

			if( disparity.disparityAlg != null ) {
				if( computedFeatures && visualize.hasLeft && visualize.hasRight ) {
					// rectify the images and compute the disparity. this is doing the feature matching too
					setProgressMessage("Rectifying using Rectification");
					long start = System.currentTimeMillis( );

					boolean success = disparity.rectifyImage();
					long end = System.currentTimeMillis( );
					long diff = end - start;
					Log.d("rect", "Time to match" + Float.toString(diff));

					if( success ) {
						Log.d("indisparity", "I am entering disparity calculation");
						try {
//							Thread.sleep(2000);
						} catch (Exception e) {
							System.out.println("Exception");
						}
						setProgressMessage("Disparity");

						disparity.computeDisparity();
						synchronized ( lockGui ) {
							disparityMin = disparity.getDisparityAlg().getMinDisparity();
							disparityMax = disparity.getDisparityAlg().getMaxDisparity();
							disparityImage.setTo(disparity.getDisparity());
							visualize.setMatches(disparity.getInliersPixel());
							visualize.forgetSelection();

							runOnUiThread(new Runnable() {
								@Override
								public void run() {
									spinnerView.setSelection(0); // switch to disparity view=2, association=0, rectification=1.
								}
							});
						}
					} else {
						synchronized ( lockGui ) {
							ImageMiscOps.fill(disparityImage,0);
						}
						runOnUiThread(new Runnable() {
							public void run() {
								Toast.makeText(DisparityActivity.this, "Disparity computation failed!", Toast.LENGTH_SHORT).show();
							}});
					}
				} else if( changeDisparityAlg != -1 && visualize.hasLeft && visualize.hasRight ) {
					// recycle the rectified image but compute the disparity using the new algorithm
					setProgressMessage("Disparity using 5-rect algorithm");
					disparity.computeDisparity();

					synchronized ( lockGui ) {
						disparityMin = disparity.getDisparityAlg().getMinDisparity();
						disparityMax = disparity.getDisparityAlg().getMaxDisparity();
						disparityImage.setTo(disparity.getDisparity());
					}
				}
			}
			hideProgressDialog();
			changeDisparityAlg = -1;
		}

		@Override
		protected void render(Canvas canvas, double imageToOutput) {
//			check if camera has been calibrated. calibrate it otherwise
			if( DemoMain.preference.intrinsic == null ) {
				canvas.restore();
				Paint paint = new Paint();
				paint.setColor(Color.RED);
				paint.setTextSize(60);
				int textLength = (int)paint.measureText("Calibrate Camera First");

				canvas.drawText("Calibrate Camera First", (canvas.getWidth() - textLength) / 2, canvas.getHeight() / 2, paint);
			} else if( activeView == DView.DISPARITY ) {
				// draw rectified image
				ConvertBitmap.grayToBitmap(disparity.rectifiedLeft, visualize.bitmapSrc, visualize.storage);
				canvas.drawBitmap(visualize.bitmapSrc,0,0,null);

				if( disparity.isDisparityAvailable() ) {
					VisualizeImageData.disparity(disparityImage,disparityMin,disparityMax,0,
							visualize.bitmapDst,visualize.storage);

					int startX = disparityImage.getWidth() + AssociationVisualize.SEPARATION;
					canvas.drawBitmap(visualize.bitmapDst,startX,0,null);
				}
			} else if( activeView == DView.RECTIFICATION ) {
				ConvertBitmap.grayToBitmap(disparity.rectifiedLeft,visualize.bitmapSrc,visualize.storage);
				ConvertBitmap.grayToBitmap(disparity.rectifiedRight,visualize.bitmapDst,visualize.storage);
				//try to save this image
//				Random rand = new Random();
//				int n = rand.nextInt(10000) + 1;
//				MediaStore.Images.Media.insertImage(getContentResolver(), visualize.bitmapSrc, "output" + Integer.toString(n) , "mySproj");
//				n = rand.nextInt(10000) + 1;
//				MediaStore.Images.Media.insertImage(getContentResolver(), visualize.bitmapDst, "output" + Integer.toString(n) , "mySproj");

				int startX = disparity.rectifiedLeft.getWidth() + AssociationVisualize.SEPARATION;
				canvas.drawBitmap(visualize.bitmapSrc,0,0,null);
				canvas.drawBitmap(visualize.bitmapDst,startX,0,null);

				if( touchY >= 0 ) {
					canvas.restore();
					canvas.drawLine(0,touchY,canvas.getWidth(),touchY,visualize.paintPoint);
				}
			} else {
				// bit of a hack to reduce memory usage
				ConvertBitmap.grayToBitmap(visualize.graySrc,visualize.bitmapSrc,visualize.storage);
				ConvertBitmap.grayToBitmap(visualize.grayDst,visualize.bitmapDst,visualize.storage);

				visualize.render(canvas,tranX,tranY,scale);
			}
		}
	}
	public void onImageGalleryClicked(View v) {
		// invoke the image gallery using an implict intent.
		Intent photoPickerIntent = new Intent(Intent.ACTION_PICK);

		// where do we want to find the data?
		File pictureDirectory = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
		String pictureDirectoryPath = pictureDirectory.getPath();
		// finally, get a URI representation
		Uri data = Uri.parse(pictureDirectoryPath);

		// set the data and type.  Get all image types.
		photoPickerIntent.setDataAndType(data, "image/*");

		// we will invoke this activity, and get something back from it.
		startActivityForResult(photoPickerIntent, IMAGE_GALLERY_REQUEST);
	}
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		if (resultCode == RESULT_OK) {
			// if we are here, everything processed successfully.
			if (requestCode == IMAGE_GALLERY_REQUEST) {
				// if we are here, we are hearing back from the image gallery.

				// the address of the image on the SD Card.
				Uri imageUri = data.getData();

				// declare a stream to read the image data from the SD Card.
				InputStream inputStream;

				// we are getting an input stream, based on the URI of the image.
				try {
					inputStream = getContentResolver().openInputStream(imageUri);

					// get a bitmap from the stream.
					Bitmap image = BitmapFactory.decodeStream(inputStream);


					// show the image to the user
					imgPicture.setImageBitmap(image);

				} catch (FileNotFoundException e) {
					e.printStackTrace();
					// show a message to the user indictating that the image is unavailable.
					Toast.makeText(this, "Unable to open image", Toast.LENGTH_LONG).show();
				}

			}
		}
	}

	enum DView {
		ASSOCIATION,
		RECTIFICATION,
		DISPARITY
	}
}