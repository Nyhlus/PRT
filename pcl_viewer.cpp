/*
 * Développé par Hadrien Greef et Sébastien Germond-Mazet
 *
 * Janvier-Avril 2015
 * Aix-Marseille Université, France
 **
 */

#include <iostream>
#include <fstream>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <string.h>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

PointCloud<PointXYZ>::Ptr mainCloud (new PointCloud<PointXYZ>);
visualization::PCLVisualizer mainViewer;

/*------ AIDE ------*/


void PRT_ShowUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" Filename [Options]\n\n"
            << "Filename: le chemin du fichier à visualiser (.PCD, .PLY)\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h          afficher cette aide\n"
            << "\n\n";

} // PRT_ShowUsage ()

void PRT_ShowControls ()
{
  std::cout << "\n\nControles du programme:\n"
            << "-------------------------------------------\n"
            << "d           sauvegarder le nouveau fichier PCD\n"
            << "j           prend un instantané de la fenêtre au format PNG\n"
            << "c           affiche les paramètres caméra\n"
            << "f           envolée jusqu'au point ciblé\n"
            << "+/-         modifie la taille des points\n"
            << "ALT +/-     zoom +/-\n"
            << "g           affiche l'échelle\n"
            << "o           switch projection parallèle/perspective\n"
            << "r           reset la caméra\n"
            << "CTRL s      sauvegarde les paramètres caméra\n"
            << "CTRL r      restaure les paramètres caméra\n"
            << "ALT s       mode stéréoscopique\n"
            << "ALT f       fenêtre maximisée\n"
            << "\n\n";

} // PRT_ShowControls ()




/*------ ECRITURE ------*/


int PRT_SavePCDCloud (const PointCloud<PointXYZ>::Ptr cloud, char * name) 
{
    if(io::savePCDFile (name, *cloud) == -1) return -1;
    std::cerr << "Saved " << cloud->points.size () << " data points to "<< name << std::endl;
    return 0;
} // PRT_SavePCDCloud ()


/*------ UTILITAIRES ------*/

void keyboardEventManager (const visualization::KeyboardEvent &event)
{
  
    if (event.keyDown ())
    {
        if      (event.getKeySym () == "d" ) 
        {
            std::cout << "Sauvegarde du fichier... " << std::endl;
            if(PRT_SavePCDCloud(mainCloud, "out.pcd") == -1) PCL_ERROR ("Echec d'écriture.\n");
        }
        else if (event.getKeySym () == "" ) 
        {
        }
    }
}


/*------ VISUALISATION ------*/


void PRT_ShowCloud (PointCloud<PointXYZ>::Ptr cloud)
{  
    mainViewer.addPointCloud (cloud, "cloud");
    
    mainViewer.registerKeyboardCallback (keyboardEventManager);
    mainViewer.setShowFPS(true);
    printf("Nombre de points du Cloud : %i\n", cloud->width * cloud->height);
    mainViewer.spin();

} // PRT_ShowCloud ()






/*------ PROGRAMME PRINCIPAL ------*/


int main (int argc, char** argv)
{
    if (argc == 1 || console::find_argument (argc, argv, "-h") >= 0)
    {
        PRT_ShowUsage (argv[0]);
        return 0;
    }
    
    // Lecture du fichier PointCloud
    string ext = string(argv[1]);
    ext = ext.substr(ext.length()-3);    

	if (ext == "pcd" || ext == "PCD")
    {
        if (io::loadPCDFile<PointXYZ>(argv[1], *mainCloud) == -1) {
            PCL_ERROR ("Impossible de lire le fichier\n");
            return(-1);
        }
    }
    else if (ext == "obj" || ext == "OBJ") {
        if (io::loadOBJFile<PointXYZ>(argv[1], *mainCloud) == -1) {
            PCL_ERROR ("Impossible de lire le fichier\n");
            return(-1);
        }
    }
    else if (ext == ".3d" || ext == ".3D") {
        ifstream inFile( argv[1], ios::in ); 
        double px,py,pz; 
        while ( inFile >>  px >>  py >> pz )
        { 
            pcl::PointXYZ basic_point; 
            basic_point.x = px; 
            basic_point.y = py; 
            basic_point.z = pz; 
            mainCloud->points.push_back(basic_point); 
        } 
        mainCloud->width = mainCloud->points.size (); 
        mainCloud->height = 1; 
    }
    else if (ext == "ply" || ext == "PLY") {
        PolygonMesh mesh;
        io::loadPolygonFilePLY(argv[1], mesh);
        io::saveVTKFile ("temp.vtk", mesh);
        vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New ();
        reader->SetFileName ("temp.vtk");
        reader->Update ();
        vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();
        vtkPolyDataToPointCloud (polydata, *mainCloud);

    }
    else {
        PRT_ShowUsage (argv[0]);
        return 0;
    }

    
    
    PRT_ShowControls();
    PRT_ShowCloud (mainCloud);

    return (0);
}
