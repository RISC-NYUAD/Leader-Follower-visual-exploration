#include "geom_extractor.h"

//Init Aruco3

void aruco3_init(double _RectangleSide, double _MarkerSize, double _MarkerSmallSize, std::string image_savepath)
{
	std::ofstream outfile;
	outfile.open(image_savepath , std::ios::out | std::ios::app);
	if (outfile.fail()){
		ROS_INFO("Could not create logfile");
		throw std::ios_base::failure(std::strerror(errno));
	}
	else{
		outfile.exceptions(outfile.exceptions() | std::ios::failbit | std::ifstream::badbit); //write fails with exception if something is wrong
		//Init ArUco -- Generate geometries
		ROS_INFO("Image file created");
		
		//Generate geometries -- Same for every Rhombi
		double distSideToSolidCenter = ((sqrt(2.0f)+1.0f)/2.0f) * _RectangleSide;
		std::vector<double> corner_final;
		//A)Lower big markers
		for(int j=0; j<8 ; j++){
			//outfile << j << ";" ;
			//1) FIND CENTER OF MARKER
			std::cout << "Marker No. " << j << " " << std::endl;
			//Create RT matrix of Marker
			Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Rotate First Eigen uses angle-axis rotation system - rotation for lower markers is over Y axis
			Eigen::AngleAxisd MarkerRotation_vector ( M_PI_4*j, Eigen::Vector3d ( 0, 1 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector);
			//Translate second -- Lower markers are higher in Y (up) axis by actualRectSideSize/2 and in Z (forward) by distSideToSolidCenter
			Eigen::Vector3d MarkerTrans(0, _RectangleSide/2, distSideToSolidCenter);
			MarkerRT.translate(MarkerTrans);
			
			//2) Create RT matrix that will get us to CORNERS
			Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3 first corner is the top left corner, followed by the top right, bottom right and bottom left.
			Eigen::Vector3d CornerTrans(0, _MarkerSize*sqrt(2)/2, 0);
			Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT_base.rotate(CornerRots);
			
			//3) Multiply RTs to get final corners and assign
			for(int k=0;k<4; k++){
				Eigen::Isometry3d CornersRT=CornersRT_base;
				Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
				CornersRT.rotate(CornerRotation_vector);
				CornersRT.translate(CornerTrans);
				Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
				std::cout << "Corner_Final: " << std::endl;
				for(int c=0;c<3;c++) {
					outfile << _Corner_Final(c,3); 
					if(k<3 || c<2) outfile << ";"; 
					std::cout << _Corner_Final(c,3) << " " ;
				}
				std::cout << std::endl;
			}
			outfile << std::endl;
			std::cout << std::endl;
		}
		//B)Middle big markers
		for(int j=0; j<4 ; j++){
			//outfile << j+8 << ";" ;
			//1) FIND CENTER OF MARKER
			std::cout << "Marker No. " << j+8 << " " << std::endl;
			//Create RT matrix of Marker
			Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Translate first 
			Eigen::Vector3d MarkerTrans(0, _RectangleSide/2, 0);
			MarkerRT.translate(MarkerTrans);
			//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
			Eigen::AngleAxisd MarkerRotation_vector ( M_PI_2*j, Eigen::Vector3d ( 0, 1 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector);
			//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
			Eigen::AngleAxisd MarkerRotation_vector2 ( -M_PI_4, Eigen::Vector3d ( 1, 0 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector2);
			//Final translate over new Z
			Eigen::Vector3d MarkerTrans2(0, 0, distSideToSolidCenter);
			MarkerRT.translate(MarkerTrans2);
			
			
			//2) Create RT matrix that will get us to CORNERS
			Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
			Eigen::Vector3d CornerTrans(0, _MarkerSize*sqrt(2)/2, 0);
			Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT_base.rotate(CornerRots);
			
			//3) Multiply RTs to get final corners and assign
			for(int k=0;k<4; k++){
				Eigen::Isometry3d CornersRT=CornersRT_base;
				Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
				CornersRT.rotate(CornerRotation_vector);
				CornersRT.translate(CornerTrans);
				Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
				std::cout << "Corner_Final: " << std::endl;
				for(int c=0;c<3;c++) {
					outfile << _Corner_Final(c,3);
					if(k<3 || c<2) outfile << ";"; 
					std::cout << _Corner_Final(c,3) << " " ;
				}
				std::cout << std::endl;
			}
			outfile << std::endl;
			std::cout << std::endl;
		}
		//C)Upper marker
		for(int j=0; j<1 ; j++){
			//outfile << j+12 << ";" ;
			//1) FIND CENTER OF MARKER
			std::cout << "Marker No. " << j+12 << " " << std::endl;
			//Create RT matrix of Marker
			Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Translate first 
			Eigen::Vector3d MarkerTrans(0, _RectangleSide/2, 0);
			MarkerRT.translate(MarkerTrans);
			//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
			Eigen::AngleAxisd MarkerRotation_vector ( M_PI_2*j, Eigen::Vector3d ( 0, 1 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector);
			//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
			Eigen::AngleAxisd MarkerRotation_vector2 ( -M_PI_2, Eigen::Vector3d ( 1, 0 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector2);
			//Final translate over new Z
			Eigen::Vector3d MarkerTrans2(0, 0, distSideToSolidCenter);
			MarkerRT.translate(MarkerTrans2);
			
			
			//2) Create RT matrix that will get us to CORNERS
			Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
			Eigen::Vector3d CornerTrans(0, _MarkerSize*sqrt(2)/2, 0);
			Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT_base.rotate(CornerRots);
			
			//3) Multiply RTs to get final corners and assign
			for(int k=0;k<4; k++){
				Eigen::Isometry3d CornersRT=CornersRT_base;
				Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
				CornersRT.rotate(CornerRotation_vector);
				CornersRT.translate(CornerTrans);
				Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
				std::cout << "Corner_Final: " << std::endl;
				for(int c=0;c<3;c++) {
					outfile << _Corner_Final(c,3); 
					if(k<3 || c<2) outfile << ";"; 
					std::cout << _Corner_Final(c,3) << " " ;
				}
				std::cout << std::endl;
			}
			outfile << std::endl;
			std::cout << std::endl;
		}
		//D)Smaller markers
		for(int j=0; j<4 ; j++){
			//outfile << j+13 << ";" ;
			//1) FIND CENTER OF MARKER
			std::cout << "Marker No. " << j+13 << " " << std::endl;
			//Create RT matrix of Marker
			Eigen::Isometry3d MarkerRT= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Translate first 
			Eigen::Vector3d MarkerTrans(0, _RectangleSide/2, 0);
			MarkerRT.translate(MarkerTrans);
			//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
			Eigen::AngleAxisd MarkerRotation_vector ( M_PI_4+M_PI_2*j, Eigen::Vector3d ( 0, 1 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector);
			//Eigen uses angle-axis rotation system - rotation for middle markers is over Y and then new X axis
			Eigen::AngleAxisd MarkerRotation_vector2 ( -35.26*M_PI/180.0, Eigen::Vector3d ( 1, 0 , 0 ) );
			MarkerRT.rotate(MarkerRotation_vector2);
			//Final translate over new Z
			Eigen::Vector3d MarkerTrans2(0, -0.003901, 0.095571); //From Solidworks
			MarkerRT.translate(MarkerTrans2);
			//Final translate over new point to go exactly to triangle center
			
			//2) Create RT matrix that will get us to CORNERS
			Eigen::Isometry3d CornersRT_base= Eigen::Isometry3d::Identity(); //it is a 4x4 matrix not 3x3
			//Corners -- For each marker, its four corners are returned in their original order (which is clockwise starting with top left). So, the first corner is the top left corner, followed by the top right, bottom right and bottom left.
			Eigen::Vector3d CornerTrans(0, _MarkerSmallSize*sqrt(2)/2, 0);
			Eigen::AngleAxisd CornerRots ( M_PI_4, Eigen::Vector3d ( 0, 0 ,1 ) );
			CornersRT_base.rotate(CornerRots);
			
			//3) Multiply RTs to get final corners and assign
			for(int k=0;k<4; k++){
				Eigen::Isometry3d CornersRT=CornersRT_base;
				Eigen::AngleAxisd CornerRotation_vector ( -M_PI_2*k, Eigen::Vector3d ( 0, 0 ,1 ) );
				CornersRT.rotate(CornerRotation_vector);
				CornersRT.translate(CornerTrans);
				Eigen::Isometry3d _Corner_Final=MarkerRT*CornersRT;
				std::cout << "Corner_Final: " << std::endl;
				for(int c=0;c<3;c++) {
					outfile << _Corner_Final(c,3); 
					if(k<3 || c<2) outfile << ";"; 
					std::cout << _Corner_Final(c,3) << " " ;
				}
				std::cout << std::endl;
			}
			outfile << std::endl;
			std::cout << std::endl;
		}
		outfile.close();
	}
}

//MAIN
int main(int argc, char **argv)
{  
	//Init ROS
	ros::init(argc, argv, "rhombi_geom_extractor");
	ros::NodeHandle nh("~");
	
	nh.getParam("RectangleSide", _RectangleSide);
	nh.getParam("MarkerSize", _MarkerSize);
	nh.getParam("MarkerSmallSize", _MarkerSmallSize);
	
	nh.getParam("geometry_savepath", image_savepath);
	nh.getParam("geometry_name", geometry_name);
	if(!image_savepath.empty() && !geometry_name.empty()){
		std::string fullpath = image_savepath + geometry_name + ".csv";
		std::cout<< "Saving to: " << fullpath << std::endl;
		std::experimental::filesystem::remove(fullpath);
		sleep(1);
		aruco3_init(_RectangleSide, _MarkerSize, _MarkerSmallSize, fullpath );
	}
	else ROS_WARN("Image path or geometry_name empty. Will not save geometry.");
	
	//LOOP START
	ros::spinOnce();
	ros::Duration(0.5).sleep();
	
	return 0;
}
