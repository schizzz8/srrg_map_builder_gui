#include <fstream>
#include <srrg_types/defs.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_core_map/cloud.h>
#include <srrg_core_map/image_map_node.h>
#include "srrg_core_map/local_map_with_traversability.h"
#include "srrg_map_builder_viewers/map_builder_viewer.h"
#include "srrg_map_builder/map_builder.h"
#include "qapplication.h"
#include <stdexcept>
#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
#include <srrg_boss/trusted_loaders.h>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace srrg_map_builder;
using namespace srrg_map_builder_gui;
using namespace srrg_boss;
using namespace srrg_core_map;


const char* banner[]= {
    "srrg_map_builder_gui_app: offline map builder to convert chunks of trajectory into a set of navigable maps",
    "usage:",
    "srrg_map_builder_gui_app [options] <boss filename>",
    "where:",
    "  -d:     [int] quadtree depth, default: 3",
    "  -range: [float] range in meters to enlarge quadtree bounding box, default: 1",
    "  -r:     [float] sparse grid resolution in meters/cell, default: 0.01",
    "  -dth:   [float] distance threshold in meters to check for local maps connectivity, default: 5",
    "  -cth:   [float] minimum percentage of overlap for two local maps to be connected, default: 0.01",
    "  -o:     [string] output file name, default: empty",
    0
};

void printBanner() {
  const char** b = banner;
  while(*b) {
    cout << *b << endl;
    b++;
  }
}

int main (int argc, char** argv) {
    if (argc<2 || ! strcmp(argv[1],"-h")) {
        printBanner();
        return 0 ;
    }
    int depth = 2;
    float range = 1;
    float resolution = 0.025;
    float distance_threshold = 5;
    float connectivity_threshold = 5;
    std::string filename = "";
    std::string output_filename="";

    int c = 1;
    while(c<argc){
        if(! strcmp(argv[c],"-d")){
            c++;
            depth = atoi(argv[c]);
        }else if(! strcmp(argv[c],"-r")){
            c++;
            resolution = atof(argv[c]);
        }else if(! strcmp(argv[c],"-range")){
            c++;
            range = atof(argv[c]);
        }else if(! strcmp(argv[c],"-dth")){
            c++;
            distance_threshold = atof(argv[c]);
        }else if(! strcmp(argv[c],"-cth")){
            c++;
            connectivity_threshold = atoi(argv[c]);
        }else if (! strcmp(argv[c], "-o")){
            c++;
            output_filename = argv[c];
        }else{
            filename = argv[c];
            break;
        }
        c++;
    }

    std::list<Serializable*> objects;
    Deserializer des;
    des.setFilePath(filename);
    Serializable* o;
    MapNodeList* lmaps = new MapNodeList;
    while ( (o = des.readObject()) ){
        LocalMap* lmap = dynamic_cast<LocalMap*>(o);
        if (lmap)
            lmaps->addElement(lmap);
        else
            objects.push_back(o);
    }

    cerr << "Running clustering algorithm for " << lmaps->size() << " maps" << endl;
    cerr << "Depth: " << depth << endl;
    cerr << "Traversability Map resolution: " << resolution << endl;

    Merger merger (depth,resolution);
    merger.computeBoundingBox(lmaps);
    merger.buildQuadtree();
    //merger.visualizeQuadtree();

    MapNodeList* clusters = merger.execute();

    for(MapNodeList::iterator it = clusters->begin(); it != clusters->end(); it++){
        LocalMapWithTraversability* lmap = dynamic_cast<LocalMapWithTraversability*> (it->get());
        if(lmap)
            objects.push_back(lmap);
    }
    cerr << "--------------------------------------------------------------------------------" << endl;

    cerr << "Executing connectivity refinement for " << clusters->size() << " local maps" << endl;
    cerr << "Distance threshold: " << distance_threshold << "m" << endl;
    Linker linker(distance_threshold,connectivity_threshold,resolution);
    linker.setInput(clusters);
    BinaryNodeRelationSet* edges = linker.execute();

    for(BinaryNodeRelationSet::iterator it = edges->begin(); it != edges->end(); it++) {
        BinaryNodeRelation* edge = dynamic_cast<BinaryNodeRelation*> (it->get());
        if(edge)
            objects.push_back(edge);
    }
    cerr << "--------------------------------------------------------------------------------" << endl;

    cerr << "Writing: " << objects.size() << " objects" << endl;
    Serializer ser;
    output_filename = filename.substr(0,filename.find("."))+"_filtered.lmaps";
    ser.setFilePath(output_filename);
    ser.setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
    for(std::list<Serializable*>::iterator it = objects.begin(); it != objects.end(); it++){
        Serializable* s = *it;
        ser.writeObject(*s);
    }

    MapNodeList nodes;
    BinaryNodeRelationSet relations;
    for(std::list<Serializable*>::iterator it = objects.begin(); it != objects.end(); it++){
        Serializable* o = *it;
        LocalMapWithTraversability* lmap = dynamic_cast<LocalMapWithTraversability*>(o);
        if(lmap)
            nodes.addElement(lmap);

        BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
        if(rel){
            LocalMapWithTraversability* from = dynamic_cast<LocalMapWithTraversability*>(rel->from());
            LocalMapWithTraversability* to = dynamic_cast<LocalMapWithTraversability*>(rel->to());
            if(from && to)
                relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
        }
    }
    QApplication app(argc, argv);
    MapBuilderViewer viewer;
    viewer.nodes = nodes;
    viewer.relations = relations;
    viewer.show();
    app.exec();
    return 1;
}
