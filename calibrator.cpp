#include <string>
#include <sstream>

#include <iostream>
#include <time.h>
#include <stdio.h>

//Used for image manipulation
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

//Used for interaction with the user
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using namespace cv;
using namespace std;


static void saveCameraParams( Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<vector<Point2f> >& imagePoints)
{
    FileStorage fs( "../CalibrationParameters", FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty())
        fs << "nrOfFrames" << (int) rvecs.size();
        
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;
    
    cout << "Calibration parameters saved to CalibrationParameters.txt" << endl;
}


/**
 * Calibrates the camera based on a set of images provided by the user
 */
void Calibrate(){
    vector<vector<Point2f> > imagePoints;    //Holds the position of the location of the inner corners of the chessboard
    Mat cameraMatrix, distCoeffs;            //Matrixes to hold the K matrix and the distortion coefficients
    Size imageSize;                          //Size of the image in pixels
    Size boardSize;                          //Tamaño del cuadriculado
    int Square_Size;                         //Size of the sides of the squares in milimeteres

    //TODO get this data from the user
    Square_Size = 30;
    boardSize.width = 4;
    boardSize.height = 7;
    //TODO ask the user for the carpet where the images are

    
    Mat view;
    vector<Point2f> pointBuf;  //Output Array for Cheesboardcorners

    //Encuentra la posición de todas las esquinas en las imágenes proveidas
    //TODO count number of images in a directory
    for(int i = 1; i < 32; i++){
        std::ostringstream oss;
        oss << "../Images/my_photo-" << i << ".jpg";
        std::string imageName = oss.str();
        
        view = imread(imageName, CV_LOAD_IMAGE_COLOR);//Loads Image
        
        if(i == 1)//Todas las imagenes deben de ser del mismo tamaño
            imageSize = view.size();  //Gets imput image size

        bool found = findChessboardCorners( view, boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    
        //If ChessboardCorners are found append to array            
        if(pointBuf.size() > 0)
            imagePoints.push_back(pointBuf); //Appends result to imagePoints
    
        pointBuf.clear();//Clears vector 
    }
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);              //Crea una matriz de ceros de 8*1

    vector<vector<Point3f> > objectPoints(1);           //Vector de Vectores que describe como se debería ver el espacio

    //Calcula como se debería ver el patrón
    for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                objectPoints[0].push_back(Point3f(float( j*Square_Size ), float( i*Square_Size ), 0));


    objectPoints.resize(imagePoints.size(), objectPoints[0]);//Crea tantas copias de ObjectPoints como de Image Points
    

    //Crea una primera aproximación de la matriz de cámara
    cameraMatrix = initCameraMatrix2D(objectPoints, imagePoints, imageSize);
    //cameraMatrix = Mat::eye(3, 3, CV_64F);

    vector<Mat> rvecs, tvecs;//Vectores de Translación y rotación para cada vista

    //Encuentra la Matrix de Cámara y los coeficientes de distorción
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);


    saveCameraParams( imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, imagePoints);
}

/**
 *Reads the calibration paramaters from the CalibrationParamters file
 *@param cameraMatrix pointer to the camara Matrix to recieve the new data
 *@param distCoeffs pointer to the matrix that will hold the distortion Coefficients
*/
void read(Mat *cameraMatrix, Mat *distCoeffs)
{
        FileStorage fs( "../CalibrationParameters", FileStorage::READ );
        fs["Camera_Matrix" ] >> *cameraMatrix;
        fs["Distortion_Coefficients"] >> *distCoeffs;
        fs.release();
}


/**
 *Shows the 3D axis onthe the image
*/
void display(){
    Mat cameraMatrix, distCoeffs;
    
    //Reads data from the calibration
    read(&cameraMatrix, &distCoeffs);
    
    cout << "Camera Matrix: " << endl << cameraMatrix << endl; 
    cout << "Distortion Coefficients: " << endl << distCoeffs << endl;
    
    
    Size boardSize;                          //Tamaño del cuadriculado
    int Square_Size;                         //Size of the sides of the squares in milimeteres

    //TODO get this data from the user
    Square_Size = 30;
    boardSize.width = 4;
    boardSize.height = 7;
    
    vector<Point3f> objectPoints;           //Vector de Vectores que describe como se debería ver el espacio
    
    //Calcula como se debería ver el patrón
    for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                objectPoints.push_back(Point3f(float( j*Square_Size ), float( i*Square_Size ), 0));
                          
    
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return;
    
    vector<Point2f> pointBuf;  //Output Array for Cheesboardcorners
    
    //3d Axis
    vector<Point3f> axis;           //Vector de Vectores que describe como se debería ver el espacio
    axis.push_back(Point3f(0, 0, 0));               //Origin
    axis.push_back(Point3f(float( 100 ), 0, 0));     //X axis end
    axis.push_back(Point3f(0, float( 100 ), 0));     //Y axis end
    axis.push_back(Point3f(0, 0, -float( 100 )));     //Z axis end
    
    vector<Point2f> imagePoints;
    
    namedWindow("edges",1);
    
    Scalar blue(255,    0,    0);
    Scalar green( 0,  255,    0);
    Scalar red(   0,    0,  255);
    
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera        
        
        bool found = findChessboardCorners( frame, boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
        
        if(found){
            Mat rvec, tvec;
            
            bool solved = solvePnP(objectPoints, pointBuf, cameraMatrix, distCoeffs, rvec, tvec); 
            
            if (solved){
                projectPoints(axis, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
                
                line(frame, imagePoints[0], imagePoints[1], green, 8);
                line(frame, imagePoints[0], imagePoints[2], red, 8);
                line(frame, imagePoints[0], imagePoints[3], blue, 8);
            }
        }
        
        imshow("axis", frame);
        if(waitKey(30) >= 0) break;
    }

}


int main(int ac, char* av[])
{
    try{
        po::options_description desc("Allowed Options");
        desc.add_options()
                ("help", "Produce help message")
                ("calibrate", "Calibrates the camera based on a set of images")
                ("display", "Shows a projection of a 3D axis on the board")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if(vm.count("help")){
            cout << desc << "\n";
            return 0;
        }
        else if(vm.count("calibrate")){
            Calibrate();
        }
        else if(vm.count("display")){
            display();       
        }
        else {
            cout << desc << "\n";
            return 0;
        }

    }
    catch(exception& e) {
        cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        cerr << "Exception of unknown type!\n";
    }

    return 0;
      
}
