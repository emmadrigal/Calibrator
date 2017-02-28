#include <string>
#include <sstream>

#include <iostream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

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



void Calibrate(){
    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;
    

    Size boardSize; //Tamaño del cuadriculado
    
    int Square_Size = 30;
    boardSize.width = 4;
    boardSize.height = 7;
    
    Mat view;
    bool blinkOutput = false;

    //Encuentra la posición de todas las esquinas en las imágenes proveidas
    for(int i = 1; i < 31; i++){
        std::ostringstream oss;
        oss << "../Images/my_photo-" << i << ".jpg";
        std::string imageName = oss.str();
        
        view = imread(imageName, CV_LOAD_IMAGE_COLOR);//Loads Image
        
        if(i == 1)//Todas las imagenes deben de ser del mismo tamaño
            imageSize = view.size();  //Gets imput image size

        vector<Point2f> pointBuf;  //Output Array
        bool found = findChessboardCorners( view, boardSize, pointBuf,
                CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    
        //If ChessboardCorners are found append to array            
        if(pointBuf.size() > 0)
            imagePoints.push_back(pointBuf); //Appends result to imagePoints
    
        pointBuf.clear();
    }
    
    distCoeffs = Mat::zeros(8, 1, CV_64F);              //Crea una matriz de ceros de 8*1

    vector<vector<Point3f> > objectPoints(1);           //Vector de Vectores que describe como se debería ver el espacio

    //Calcula como se debería ver el patrón
    for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j ){
                objectPoints[0].push_back(Point3f(float( j*Square_Size ), float( i*Square_Size ), 0));
                Point3f(float( j*Square_Size ), float( i*Square_Size ), 0);
            }


    objectPoints.resize(imagePoints.size(), objectPoints[0]);//Crea tantas copias de ObjectPoints como de Image Points
    

    //Crea una primera aproximación de la matriz de cámara
    cameraMatrix = initCameraMatrix2D(objectPoints, imagePoints, imageSize);
    //cameraMatrix = Mat::eye(3, 3, CV_64F);

    vector<Mat> rvecs, tvecs;//Vectores de Translación y rotación para cada vista

    //Encuentra la Matrix de Cámara y los coeficientes de distorción
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);


    saveCameraParams( imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, imagePoints);
}

int main(int ac, char* av[])
{
    try{
        po::options_description desc("Allowed Options");
        desc.add_options()
                ("help", "Produce help message")
                ("calibrate", "Calibrates the camera based on a carpet full of images")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(ac, av, desc), vm);
        po::notify(vm);

        if(vm.count("help")){
            cout << desc << "\n";
            return 0;
        }
        
        if(vm.count("calibrate")){
            Calibrate();
        } else {
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
