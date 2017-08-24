#include <intensity_map_lib/intensity_map_lib.h>
#include <boost/foreach.hpp>

#define SCORE_TRESH 200

intensity_map::intensity_map(size_t markers_size = 0)
{
	std::vector<Eigen::Vector3f> map_markers;
	map_markers.resize(markers_size);
	size_t global_iterator = 0;
	boost::property_tree::ptree xml_tree;
	boost::property_tree::ptree save_xml_tree;
	const boost::property_tree::ptree& marker_tree = save_xml_tree;
	std::ostringstream marker_path;
	Eigen::Vector3f tmp_marker;

}

intensity_map::intensity_map()
{
	intensity_map(0);
}
intensity_map::~intensity_map()
{

}
bool intensity_map::load_map(const std::string &file_path, std::vector<Eigen::Vector3f> &map_markers, size_t &used_markers_count)
{

	boost::property_tree::read_xml(file_path, xml_tree);
	used_markers_count = 0;
	BOOST_FOREACH(const boost::property_tree::ptree::value_type &v, xml_tree.get_child("map"))
    {
    	tmp_marker.head(2) << v.second.get<float>("x"), v.second.get<float>("y");
    	tmp_marker(2) = v.second.get<float>("score");
    	map_markers.at(used_markers_count) = tmp_marker;
    	++used_markers_count;
    }
    return true;
}
bool intensity_map::save_map(const std::string &file_path, const std::vector<Eigen::Vector3f> &map_markers, size_t &used_markers_count)
{
    for (global_iterator = 0; global_iterator < used_markers_count; global_iterator++ )
    {
    	if (map_markers.at(global_iterator)(2) > SCORE_TRESH)
    	{
	        marker_path.str("");
	        marker_path << "marker_" << global_iterator;

	        //marker_tree = xml_tree.add_child(marker_path.str(), boost::property_tree::ptree{});
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".x";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(0));
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".y";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(1));
	        marker_path.str("");
	        marker_path << "map.marker_"<<global_iterator<<".score";
	        save_xml_tree.put(marker_path.str(), map_markers.at(global_iterator)(2));
    	}

    }

    boost::property_tree::write_xml(file_path, save_xml_tree,
        std::locale(),
        boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1u)); 
	
	return true;
}

bool intensity_map::getMarkers(std::vector<Eigen::Vector3f> &markers)
{
	markers = map_markers;
	return true;
}