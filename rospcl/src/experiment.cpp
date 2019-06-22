#include <iostream> //标准输入/输出
#include <boost/thread/thread.hpp> //多线程
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h> //深度图有关头文件
#include <pcl/io/pcd_io.h> //pcd文件输入/输出
#include <pcl/io/png_io.h>
#include <pcl/visualization/range_image_visualizer.h> //深度图可视化
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/console/parse.h> //命令行参数解析
#include <stdlib.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


//define the grasp model
float length = 0.06;// grasp length
float width_radius = 0.005; // grasp width
float finger = 0.03;// finger space

// Mutex: //
boost::mutex cloud_mutex;
int main()
{
    // put in .pcd 
    std::string filename("/home/zsm/catkin_ws/src/rospcl/src/rgbtest222.pcd");
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_grasp(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<int>angle_grasp;
    // .pcd >> cloud
    if (pcl::io::loadPCDFile(filename, *cloud))
    {
        std::cerr << "ERROR: Cannot open file " << filename << "! Aborting..." << std::endl;
        return 0;
    }
  
    std::cout <<"cloud.size = "<< cloud->points.size() << std::endl;

    // pieces cloud ,
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*cloud,min,max);
    std::cout<<"min.z = "<<min.z<<endl;
    for(int xxx = 0;xxx<cloud->points.size();xxx++)
    {
        if(cloud->points[xxx].z > min.z+0.08 && cloud->points[xxx].z < min.z+0.095)
        {
            pcl::PointXYZ point;
            point.x = cloud->points[xxx].x;
            point.y = cloud->points[xxx].y;
            point.z = 0.3;
            cloud1->points.push_back(point);
        }
    }
    cloud1->width = (uint32_t) cloud1->points.size();
    cloud1->height = 1;
    std::cout <<"cloud1.size = "<< cloud1->points.size() << std::endl;//get first pieces -- cloud1

    //pcl::KdTreeFLANN<pcl::PointXYZ> tree_root;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_root(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_root->setInputCloud(cloud1);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01);
    ec.setMinClusterSize(200);
    ec.setSearchMethod(tree_root);
    ec.setInputCloud(cloud1);
    ec.extract(cluster_indices);
    std::cout <<"cluster_indices.size = "<< cluster_indices.size() << endl;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end();++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for(std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end();pit++)
        cloud_cluster->points.push_back(cloud1->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
    
    
    // get roi of each chunk cloud from cloud1
    pcl::PointXYZ min1;
    pcl::PointXYZ max1;
    pcl::getMinMax3D(*cloud_cluster,min1,max1);
    std::cout<<"min.x   max.x   "<<min1.x<<"  "<<max1.x<<endl;
    std::cout<<"min.y   max.y   "<<min1.y<<"  "<<max1.y<<endl;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_cluster);
   // kdtree.setInputCloud(cloud1); // use cloud1 can avoid collision with other cloud_cluster;
    for(float roi_x = min1.x;roi_x<max1.x;roi_x+=0.005)
    {
        for(float roi_y = min1.y;roi_y<max1.y;roi_y+=0.005)
        {
            float weight = 0.101;
            // std::cout<< "weight = "<< weight<<endl;
            pcl::PointXYZ graspcentre;
            graspcentre.x = roi_x;
            graspcentre.y = roi_y;
            graspcentre.z = 0.25;
            cloud2->points.push_back(graspcentre);
            graspcentre.z = 0.3;
            
            float radius = finger*0.5;
            // 0  cos0 = 1,sin0 = 0
            pcl::PointXYZ sp_a;
            sp_a.x = (length+finger)*0.5+graspcentre.x;
            sp_a.y = graspcentre.y;
            sp_a.z = 0.3;
            std::vector<int>spindex_a;
            std::vector<float>spdistance_a;
            pcl::PointXYZ sp_b;
            sp_b.x = -(length+finger)*0.5+graspcentre.x;
            sp_b.y = graspcentre.y;
            sp_b.z = 0.3;
            std::vector<int>spindex_b;
            std::vector<float>spdistance_b;
            if(kdtree.radiusSearch(sp_a,radius,spindex_a,spdistance_a)+ kdtree.radiusSearch(sp_b,radius,spindex_b,spdistance_b)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_aa;
                sp_aa.x = length*0.5*0.33+graspcentre.x;
                sp_aa.y = graspcentre.y;
                sp_aa.z = 0.3;
                std::vector<int>spindex_aa;
                std::vector<float>spdistance_aa;
                pcl::PointXYZ sp_bb;
                sp_bb.x = -length*0.5*0.33+graspcentre.x;
                sp_bb.y = graspcentre.y;
                sp_bb.z = 0.3;
                std::vector<int>spindex_bb;
                std::vector<float>spdistance_bb;
                pcl::PointXYZ sp_aaa;
                sp_aaa.x = length*0.5*0.67+graspcentre.x;
                sp_aaa.y = graspcentre.y;
                sp_aaa.z = 0.3;
                std::vector<int>spindex_aaa;
                std::vector<float>spdistance_aaa;
                pcl::PointXYZ sp_bbb;
                sp_bbb.x = -length*0.5*0.67+graspcentre.x;
                sp_bbb.y = graspcentre.y;
                sp_bbb.z = 0.3;
                std::vector<int>spindex_bbb;
                std::vector<float>spdistance_bbb;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_aa,width_radius,spindex_aa,spdistance_aa)+kdtree.radiusSearch(sp_bb,width_radius,spindex_bb,spdistance_bb)+kdtree.radiusSearch(sp_aaa,width_radius,spindex_aaa,spdistance_aaa)+kdtree.radiusSearch(sp_bbb,width_radius,spindex_bbb,spdistance_bbb))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.24;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }
                
            }
            // 15  cos15 = 0.966,sin15 = 0.259
            pcl::PointXYZ sp_c;
            sp_c.x = (length+finger)*0.5*0.966+graspcentre.x;
            sp_c.y = -(length+finger)*0.5*0.259+graspcentre.y;
            sp_c.z = 0.3;
            std::vector<int>spindex_c;
            std::vector<float>spdistance_c;
            pcl::PointXYZ sp_d;
            sp_d.x = -(length+finger)*0.5*0.966+graspcentre.x;
            sp_d.y = (length+finger)*0.5*0.259+graspcentre.y;
            sp_d.z = 0.3;
            std::vector<int>spindex_d;
            std::vector<float>spdistance_d;
            if(kdtree.radiusSearch(sp_c,radius,spindex_c,spdistance_c)+ kdtree.radiusSearch(sp_d,radius,spindex_d,spdistance_d)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_cc;
                sp_cc.x = length*0.5*0.966*0.33+graspcentre.x;
                sp_cc.y = -length*0.5*0.259*0.33+graspcentre.y;
                sp_cc.z = 0.3;
                std::vector<int>spindex_cc;
                std::vector<float>spdistance_cc;
                pcl::PointXYZ sp_dd;
                sp_dd.x = -length*0.5*0.966*0.33+graspcentre.x;
                sp_dd.y = length*0.5*0.259*0.33+graspcentre.y;
                sp_dd.z = 0.3;
                std::vector<int>spindex_dd;
                std::vector<float>spdistance_dd;
                pcl::PointXYZ sp_ccc;
                sp_ccc.x = length*0.5*0.966*0.67+graspcentre.x;
                sp_ccc.y = -length*0.5*0.259*0.67+graspcentre.y;
                sp_ccc.z = 0.3;
                std::vector<int>spindex_ccc;
                std::vector<float>spdistance_ccc;
                pcl::PointXYZ sp_ddd;
                sp_ddd.x = -length*0.5*0.966*0.67+graspcentre.x;
                sp_ddd.y = length*0.5*0.259*0.67+graspcentre.y;
                sp_ddd.z = 0.3;
                std::vector<int>spindex_ddd;
                std::vector<float>spdistance_ddd;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_cc,width_radius,spindex_cc,spdistance_cc)+kdtree.radiusSearch(sp_dd,width_radius,spindex_dd,spdistance_dd)+kdtree.radiusSearch(sp_ccc,width_radius,spindex_ccc,spdistance_ccc)+kdtree.radiusSearch(sp_ddd,width_radius,spindex_ddd,spdistance_ddd))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.23;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            // 30  cos30 = 0.866,sin30 = 0.5
            pcl::PointXYZ sp_e;
            sp_e.x = (length+finger)*0.5*0.866+graspcentre.x;
            sp_e.y = -(length+finger)*0.5*0.5+graspcentre.y;
            sp_e.z = 0.3;
            std::vector<int>spindex_e;
            std::vector<float>spdistance_e;
            pcl::PointXYZ sp_f;
            sp_f.x = -(length+finger)*0.5*0.866+graspcentre.x;
            sp_f.y = (length+finger)*0.5*0.5+graspcentre.y;
            sp_f.z = 0.3;
            std::vector<int>spindex_f;
            std::vector<float>spdistance_f;
            if(kdtree.radiusSearch(sp_e,radius,spindex_e,spdistance_e)+ kdtree.radiusSearch(sp_f,radius,spindex_f,spdistance_f)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_ee;
                sp_ee.x = length*0.5*0.866*0.33+graspcentre.x;
                sp_ee.y = -length*0.5*0.5*0.33+graspcentre.y;
                sp_ee.z = 0.3;
                std::vector<int>spindex_ee;
                std::vector<float>spdistance_ee;
                pcl::PointXYZ sp_ff;
                sp_ff.x = -length*0.5*0.866*0.33+graspcentre.x;
                sp_ff.y = length*0.5*0.5*0.33+graspcentre.y;
                sp_ff.z = 0.3;
                std::vector<int>spindex_ff;
                std::vector<float>spdistance_ff;
                pcl::PointXYZ sp_eee;
                sp_eee.x = length*0.5*0.866*0.67+graspcentre.x;
                sp_eee.y = -length*0.5*0.5*0.67+graspcentre.y;
                sp_eee.z = 0.3;
                std::vector<int>spindex_eee;
                std::vector<float>spdistance_eee;
                pcl::PointXYZ sp_fff;
                sp_fff.x = -length*0.5*0.866*0.67+graspcentre.x;
                sp_fff.y = length*0.5*0.5*0.67+graspcentre.y;
                sp_fff.z = 0.3;
                std::vector<int>spindex_fff;
                std::vector<float>spdistance_fff;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_ee,width_radius,spindex_ee,spdistance_ee)+kdtree.radiusSearch(sp_ff,width_radius,spindex_ff,spdistance_ff)+kdtree.radiusSearch(sp_eee,width_radius,spindex_eee,spdistance_eee)+kdtree.radiusSearch(sp_fff,width_radius,spindex_fff,spdistance_fff))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.22;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }
            }
            // 45  cos45 = 0.707,sin30 = 0.707
            pcl::PointXYZ sp_g;
            sp_g.x = (length+finger)*0.5*0.707+graspcentre.x;
            sp_g.y = -(length+finger)*0.5*0.707+graspcentre.y;
            sp_g.z = 0.3;
            std::vector<int>spindex_g;
            std::vector<float>spdistance_g;
            pcl::PointXYZ sp_h;
            sp_h.x = -(length+finger)*0.5*0.707+graspcentre.x;
            sp_h.y = (length+finger)*0.5*0.707+graspcentre.y;
            sp_h.z = 0.3;
            std::vector<int>spindex_h;
            std::vector<float>spdistance_h;
            if(kdtree.radiusSearch(sp_g,radius,spindex_g,spdistance_g)+ kdtree.radiusSearch(sp_h,radius,spindex_h,spdistance_h)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_gg;
                sp_gg.x = length*0.5*0.707*0.33+graspcentre.x;
                sp_gg.y = -length*0.5*0.707*0.33+graspcentre.y;
                sp_gg.z = 0.3;
                std::vector<int>spindex_gg;
                std::vector<float>spdistance_gg;
                pcl::PointXYZ sp_hh;
                sp_hh.x = -length*0.5*0.707*0.33+graspcentre.x;
                sp_hh.y = length*0.5*0.707*0.33+graspcentre.y;
                sp_hh.z = 0.3;
                std::vector<int>spindex_hh;
                std::vector<float>spdistance_hh;
                pcl::PointXYZ sp_ggg;
                sp_ggg.x = length*0.5*0.707*0.67+graspcentre.x;
                sp_ggg.y = -length*0.5*0.707*0.67+graspcentre.y;
                sp_ggg.z = 0.3;
                std::vector<int>spindex_ggg;
                std::vector<float>spdistance_ggg;
                pcl::PointXYZ sp_hhh;
                sp_hhh.x = -length*0.5*0.707*0.67+graspcentre.x;
                sp_hhh.y = length*0.5*0.707*0.67+graspcentre.y;
                sp_hhh.z = 0.3;
                std::vector<int>spindex_hhh;
                std::vector<float>spdistance_hhh;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_gg,width_radius,spindex_gg,spdistance_gg)+kdtree.radiusSearch(sp_hh,width_radius,spindex_hh,spdistance_hh)+kdtree.radiusSearch(sp_ggg,width_radius,spindex_ggg,spdistance_ggg)+kdtree.radiusSearch(sp_hhh,width_radius,spindex_hhh,spdistance_hhh))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.21;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);                  
                }
            }
            // 60  cos60 = 0.5,sin60 = 0.866
            pcl::PointXYZ sp_i;
            sp_i.x = (length+finger)*0.5*0.5+graspcentre.x;
            sp_i.y = -(length+finger)*0.5*0.866+graspcentre.y;
            sp_i.z = 0.3;
            std::vector<int>spindex_i;
            std::vector<float>spdistance_i;
            pcl::PointXYZ sp_j;
            sp_j.x = -(length+finger)*0.5*0.5+graspcentre.x;
            sp_j.y = (length+finger)*0.5*0.866+graspcentre.y;
            sp_j.z = 0.3;
            std::vector<int>spindex_j;
            std::vector<float>spdistance_j;
            if(kdtree.radiusSearch(sp_i,radius,spindex_i,spdistance_i)+ kdtree.radiusSearch(sp_j,radius,spindex_j,spdistance_j)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_ii;
                sp_ii.x = length*0.5*0.5*0.33+graspcentre.x;
                sp_ii.y = -length*0.5*0.866*0.33+graspcentre.y;
                sp_ii.z = 0.3;
                std::vector<int>spindex_ii;
                std::vector<float>spdistance_ii;
                pcl::PointXYZ sp_jj;
                sp_jj.x = -length*0.5*0.5*0.33+graspcentre.x;
                sp_jj.y = length*0.5*0.866*0.33+graspcentre.y;
                sp_jj.z = 0.3;
                std::vector<int>spindex_jj;
                std::vector<float>spdistance_jj;
                pcl::PointXYZ sp_iii;
                sp_iii.x = length*0.5*0.5*0.67+graspcentre.x;
                sp_iii.y = -length*0.5*0.866*0.67+graspcentre.y;
                sp_iii.z = 0.3;
                std::vector<int>spindex_iii;
                std::vector<float>spdistance_iii;
                pcl::PointXYZ sp_jjj;
                sp_jjj.x = -length*0.5*0.5*0.67+graspcentre.x;
                sp_jjj.y = length*0.5*0.866*0.67+graspcentre.y;
                sp_jjj.z = 0.3;
                std::vector<int>spindex_jjj;
                std::vector<float>spdistance_jjj;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_ii,width_radius,spindex_ii,spdistance_ii)+kdtree.radiusSearch(sp_jj,width_radius,spindex_jj,spdistance_jj)+kdtree.radiusSearch(sp_iii,width_radius,spindex_iii,spdistance_iii)+kdtree.radiusSearch(sp_jjj,width_radius,spindex_jjj,spdistance_jjj))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.20;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            // 75  cos75 = 0.259,sin75 = 0.966
            pcl::PointXYZ sp_k;
            sp_k.x = (length+finger)*0.5*0.259+graspcentre.x;
            sp_k.y = -(length+finger)*0.5*0.966+graspcentre.y;
            sp_k.z = 0.3;
            std::vector<int>spindex_k;
            std::vector<float>spdistance_k;
            pcl::PointXYZ sp_l;
            sp_l.x = -(length+finger)*0.5*0.259+graspcentre.x;
            sp_l.y = (length+finger)*0.5*0.966+graspcentre.y;
            sp_l.z = 0.3;
            std::vector<int>spindex_l;
            std::vector<float>spdistance_l;
            if(kdtree.radiusSearch(sp_k,radius,spindex_k,spdistance_k)+ kdtree.radiusSearch(sp_l,radius,spindex_l,spdistance_l)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_kk;
                sp_kk.x = length*0.5*0.259*0.33+graspcentre.x;
                sp_kk.y = -length*0.5*0.966*0.33+graspcentre.y;
                sp_kk.z = 0.3;
                std::vector<int>spindex_kk;
                std::vector<float>spdistance_kk;
                pcl::PointXYZ sp_ll;
                sp_ll.x = -length*0.5*0.259*0.33+graspcentre.x;
                sp_ll.y = length*0.5*0.966*0.33+graspcentre.y;
                sp_ll.z = 0.3;
                std::vector<int>spindex_ll;
                std::vector<float>spdistance_ll;
                pcl::PointXYZ sp_kkk;
                sp_kkk.x = length*0.5*0.259*0.67+graspcentre.x;
                sp_kkk.y = -length*0.5*0.966*0.67+graspcentre.y;
                sp_kkk.z = 0.3;
                std::vector<int>spindex_kkk;
                std::vector<float>spdistance_kkk;
                pcl::PointXYZ sp_lll;
                sp_lll.x = -length*0.5*0.259*0.67+graspcentre.x;
                sp_lll.y = length*0.5*0.966*0.67+graspcentre.y;
                sp_lll.z = 0.3;
                std::vector<int>spindex_lll;
                std::vector<float>spdistance_lll;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_kk,width_radius,spindex_kk,spdistance_kk)+kdtree.radiusSearch(sp_ll,width_radius,spindex_ll,spdistance_ll)+kdtree.radiusSearch(sp_kkk,width_radius,spindex_kkk,spdistance_kkk)+kdtree.radiusSearch(sp_lll,width_radius,spindex_lll,spdistance_lll))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.19;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            // 90  cos90 = 0,sin90 = 1
            pcl::PointXYZ sp_m;
            sp_m.x = graspcentre.x;
            sp_m.y = -(length+finger)*0.5+graspcentre.y;
            sp_m.z = 0.3;
            std::vector<int>spindex_m;
            std::vector<float>spdistance_m;
            pcl::PointXYZ sp_n;
            sp_n.x = graspcentre.x;
            sp_n.y = (length+finger)*0.5+graspcentre.y;
            sp_n.z = 0.3;
            std::vector<int>spindex_n;
            std::vector<float>spdistance_n;
            if(kdtree.radiusSearch(sp_m,radius,spindex_m,spdistance_m)+ kdtree.radiusSearch(sp_n,radius,spindex_n,spdistance_n)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_mm;
                sp_mm.x = graspcentre.x;
                sp_mm.y = -length*0.5*0.33+graspcentre.y;
                sp_mm.z = 0.3;
                std::vector<int>spindex_mm;
                std::vector<float>spdistance_mm;
                pcl::PointXYZ sp_nn;
                sp_nn.x = graspcentre.x;
                sp_nn.y = length*0.5*0.33+graspcentre.y;
                sp_nn.z = 0.3;
                std::vector<int>spindex_nn;
                std::vector<float>spdistance_nn;
                pcl::PointXYZ sp_mmm;
                sp_mmm.x = graspcentre.x;
                sp_mmm.y = -length*0.5*0.67+graspcentre.y;
                sp_mmm.z = 0.3;
                std::vector<int>spindex_mmm;
                std::vector<float>spdistance_mmm;
                pcl::PointXYZ sp_nnn;
                sp_nnn.x = graspcentre.x;
                sp_nnn.y = length*0.5*0.67+graspcentre.y;
                sp_nnn.z = 0.3;
                std::vector<int>spindex_nnn;
                std::vector<float>spdistance_nnn;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_mm,width_radius,spindex_mm,spdistance_mm)+kdtree.radiusSearch(sp_nn,width_radius,spindex_nn,spdistance_nn)+kdtree.radiusSearch(sp_mmm,width_radius,spindex_mmm,spdistance_mmm)+kdtree.radiusSearch(sp_nnn,width_radius,spindex_nnn,spdistance_nnn))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.18;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }
            }
            // 105  cos105 = -0.259,sin75 = 0.966
            pcl::PointXYZ sp_o;
            sp_o.x = -(length+finger)*0.5*0.259+graspcentre.x;
            sp_o.y = -(length+finger)*0.5*0.966+graspcentre.y;
            sp_o.z = 0.3;
            std::vector<int>spindex_o;
            std::vector<float>spdistance_o;
            pcl::PointXYZ sp_p;
            sp_p.x = (length+finger)*0.5*0.259+graspcentre.x;
            sp_p.y = (length+finger)*0.5*0.966+graspcentre.y;
            sp_p.z = 0.3;
            std::vector<int>spindex_p;
            std::vector<float>spdistance_p;
            if(kdtree.radiusSearch(sp_o,radius,spindex_o,spdistance_o)+ kdtree.radiusSearch(sp_p,radius,spindex_p,spdistance_p)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_oo;
                sp_oo.x = -length*0.5*0.259*0.33+graspcentre.x;
                sp_oo.y = -length*0.5*0.966*0.33+graspcentre.y;
                sp_oo.z = 0.3;
                std::vector<int>spindex_oo;
                std::vector<float>spdistance_oo;
                pcl::PointXYZ sp_pp;
                sp_pp.x = length*0.5*0.259*0.33+graspcentre.x;
                sp_pp.y = length*0.5*0.966*0.33+graspcentre.y;
                sp_pp.z = 0.3;
                std::vector<int>spindex_pp;
                std::vector<float>spdistance_pp;
                pcl::PointXYZ sp_ooo;
                sp_ooo.x = -length*0.5*0.259*0.67+graspcentre.x;
                sp_ooo.y = -length*0.5*0.966*0.67+graspcentre.y;
                sp_ooo.z = 0.3;
                std::vector<int>spindex_ooo;
                std::vector<float>spdistance_ooo;
                pcl::PointXYZ sp_ppp;
                sp_ppp.x = length*0.5*0.259*0.67+graspcentre.x;
                sp_ppp.y = length*0.5*0.966*0.67+graspcentre.y;
                sp_ppp.z = 0.3;
                std::vector<int>spindex_ppp;
                std::vector<float>spdistance_ppp;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_oo,width_radius,spindex_oo,spdistance_oo)+kdtree.radiusSearch(sp_pp,width_radius,spindex_pp,spdistance_pp)+kdtree.radiusSearch(sp_ooo,width_radius,spindex_ooo,spdistance_ooo)+kdtree.radiusSearch(sp_ppp,width_radius,spindex_ppp,spdistance_ppp))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.17;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            // 120  cos120 = -0.5,sin60 = 0.866
            pcl::PointXYZ sp_q;
            sp_q.x = -(length+finger)*0.5*0.5+graspcentre.x;
            sp_q.y = -(length+finger)*0.5*0.866+graspcentre.y;
            sp_q.z = 0.3;
            std::vector<int>spindex_q;
            std::vector<float>spdistance_q;
            pcl::PointXYZ sp_r;
            sp_r.x = (length+finger)*0.5*0.5+graspcentre.x;
            sp_r.y = (length+finger)*0.5*0.866+graspcentre.y;
            sp_r.z = 0.3;
            std::vector<int>spindex_r;
            std::vector<float>spdistance_r;
            if(kdtree.radiusSearch(sp_q,radius,spindex_q,spdistance_q)+ kdtree.radiusSearch(sp_r,radius,spindex_r,spdistance_r)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_qq;
                sp_qq.x = -length*0.5*0.5*0.33+graspcentre.x;
                sp_qq.y = -length*0.5*0.866*0.33+graspcentre.y;
                sp_qq.z = 0.3;
                std::vector<int>spindex_qq;
                std::vector<float>spdistance_qq;
                pcl::PointXYZ sp_rr;
                sp_rr.x = length*0.5*0.5*0.33+graspcentre.x;
                sp_rr.y = length*0.5*0.866*0.33+graspcentre.y;
                sp_rr.z = 0.3;
                std::vector<int>spindex_rr;
                std::vector<float>spdistance_rr;
                pcl::PointXYZ sp_qqq;
                sp_qqq.x = -length*0.5*0.5*0.67+graspcentre.x;
                sp_qqq.y = -length*0.5*0.866*0.67+graspcentre.y;
                sp_qqq.z = 0.3;
                std::vector<int>spindex_qqq;
                std::vector<float>spdistance_qqq;
                pcl::PointXYZ sp_rrr;
                sp_rrr.x = length*0.5*0.5*0.67+graspcentre.x;
                sp_rrr.y = length*0.5*0.866*0.67+graspcentre.y;
                sp_rrr.z = 0.3;
                std::vector<int>spindex_rrr;
                std::vector<float>spdistance_rrr;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_qq,width_radius,spindex_qq,spdistance_qq)+kdtree.radiusSearch(sp_rr,width_radius,spindex_rr,spdistance_rr)+kdtree.radiusSearch(sp_qqq,width_radius,spindex_qqq,spdistance_qqq)+kdtree.radiusSearch(sp_rrr,width_radius,spindex_rrr,spdistance_rrr))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.16;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }
                
            }
            // 135  cos135 = -0.707,sin135 = 0.707
            pcl::PointXYZ sp_s;
            sp_s.x = -(length+finger)*0.5*0.707+graspcentre.x;
            sp_s.y = -(length+finger)*0.5*0.707+graspcentre.y;
            sp_s.z = 0.3;
            std::vector<int>spindex_s;
            std::vector<float>spdistance_s;
            pcl::PointXYZ sp_u;
            sp_u.x = (length+finger)*0.5*0.707+graspcentre.x;
            sp_u.y = (length+finger)*0.5*0.707+graspcentre.y;
            sp_u.z = 0.3;
            std::vector<int>spindex_u;
            std::vector<float>spdistance_u;
            if(kdtree.radiusSearch(sp_s,radius,spindex_s,spdistance_s)+ kdtree.radiusSearch(sp_u,radius,spindex_u,spdistance_u)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_ss;
                sp_ss.x = -length*0.5*0.707*0.33+graspcentre.x;
                sp_ss.y = -length*0.5*0.707*0.33+graspcentre.y;
                sp_ss.z = 0.3;
                std::vector<int>spindex_ss;
                std::vector<float>spdistance_ss;
                pcl::PointXYZ sp_uu;
                sp_uu.x = length*0.5*0.707*0.33+graspcentre.x;
                sp_uu.y = length*0.5*0.707*0.33+graspcentre.y;
                sp_uu.z = 0.3;
                std::vector<int>spindex_uu;
                std::vector<float>spdistance_uu;
                pcl::PointXYZ sp_sss;
                sp_sss.x = -length*0.5*0.707*0.67+graspcentre.x;
                sp_sss.y = -length*0.5*0.707*0.67+graspcentre.y;
                sp_sss.z = 0.3;
                std::vector<int>spindex_sss;
                std::vector<float>spdistance_sss;
                pcl::PointXYZ sp_uuu;
                sp_uuu.x = length*0.5*0.707*0.67+graspcentre.x;
                sp_uuu.y = length*0.5*0.707*0.67+graspcentre.y;
                sp_uuu.z = 0.3;
                std::vector<int>spindex_uuu;
                std::vector<float>spdistance_uuu;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_ss,width_radius,spindex_ss,spdistance_ss)+kdtree.radiusSearch(sp_uu,width_radius,spindex_uu,spdistance_uu)+kdtree.radiusSearch(sp_sss,width_radius,spindex_sss,spdistance_sss)+kdtree.radiusSearch(sp_uuu,width_radius,spindex_uuu,spdistance_uuu))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.15;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            // 150  cos150 = -0.866,sin150 = 0.5
            pcl::PointXYZ sp_v;
            sp_v.x = -(length+finger)*0.5*0.866+graspcentre.x;
            sp_v.y = -(length+finger)*0.5*0.5+graspcentre.y;
            sp_v.z = 0.3;
            std::vector<int>spindex_v;
            std::vector<float>spdistance_v;
            pcl::PointXYZ sp_w;
            sp_w.x = (length+finger)*0.5*0.866+graspcentre.x;
            sp_w.y = (length+finger)*0.5*0.5+graspcentre.y;
            sp_w.z = 0.3;
            std::vector<int>spindex_w;
            std::vector<float>spdistance_w;
            if(kdtree.radiusSearch(sp_v,radius,spindex_v,spdistance_v)+ kdtree.radiusSearch(sp_w,radius,spindex_w,spdistance_w)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_vv;
                sp_vv.x = -length*0.5*0.866*0.33+graspcentre.x;
                sp_vv.y = -length*0.5*0.5*0.33+graspcentre.y;
                sp_vv.z = 0.3;
                std::vector<int>spindex_vv;
                std::vector<float>spdistance_vv;
                pcl::PointXYZ sp_ww;
                sp_ww.x = length*0.5*0.866*0.33+graspcentre.x;
                sp_ww.y = length*0.5*0.5*0.33+graspcentre.y;
                sp_ww.z = 0.3;
                std::vector<int>spindex_ww;
                std::vector<float>spdistance_ww;
                pcl::PointXYZ sp_vvv;
                sp_vvv.x = -length*0.5*0.866*0.67+graspcentre.x;
                sp_vvv.y = -length*0.5*0.5*0.67+graspcentre.y;
                sp_vvv.z = 0.3;
                std::vector<int>spindex_vvv;
                std::vector<float>spdistance_vvv;
                pcl::PointXYZ sp_www;
                sp_www.x = length*0.5*0.866*0.67+graspcentre.x;
                sp_www.y = length*0.5*0.5*0.67+graspcentre.y;
                sp_www.z = 0.3;
                std::vector<int>spindex_www;
                std::vector<float>spdistance_www;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_vv,width_radius,spindex_vv,spdistance_vv)+kdtree.radiusSearch(sp_ww,width_radius,spindex_ww,spdistance_ww)+kdtree.radiusSearch(sp_vvv,width_radius,spindex_vvv,spdistance_vvv)+kdtree.radiusSearch(sp_www,width_radius,spindex_www,spdistance_www))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.14;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            // 165  cos165 = -0.966,sin165 = 0.259
            pcl::PointXYZ sp_x;
            sp_x.x = -(length+finger)*0.5*0.966+graspcentre.x;
            sp_x.y = -(length+finger)*0.5*0.259+graspcentre.y;
            sp_x.z = 0.3;
            std::vector<int>spindex_x;
            std::vector<float>spdistance_x;
            pcl::PointXYZ sp_y;
            sp_y.x = (length+finger)*0.5*0.966+graspcentre.x;
            sp_y.y = (length+finger)*0.5*0.259+graspcentre.y;
            sp_y.z = 0.3;
            std::vector<int>spindex_y;
            std::vector<float>spdistance_y;
            if(kdtree.radiusSearch(sp_x,radius,spindex_x,spdistance_x)+ kdtree.radiusSearch(sp_y,radius,spindex_y,spdistance_y)==0 )
            {
                std::vector<int>spindex_graspcentre;
                std::vector<float>spdistance_graspcentre;
                pcl::PointXYZ sp_xx;
                sp_xx.x = -length*0.5*0.866*0.33+graspcentre.x;
                sp_xx.y = -length*0.5*0.5*0.33+graspcentre.y;
                sp_xx.z = 0.3;
                std::vector<int>spindex_xx;
                std::vector<float>spdistance_xx;
                pcl::PointXYZ sp_yy;
                sp_yy.x = length*0.5*0.866*0.33+graspcentre.x;
                sp_yy.y = length*0.5*0.5*0.33+graspcentre.y;
                sp_yy.z = 0.3;
                std::vector<int>spindex_yy;
                std::vector<float>spdistance_yy;
                pcl::PointXYZ sp_xxx;
                sp_xxx.x = -length*0.5*0.866*0.67+graspcentre.x;
                sp_xxx.y = -length*0.5*0.5*0.67+graspcentre.y;
                sp_xxx.z = 0.3;
                std::vector<int>spindex_xxx;
                std::vector<float>spdistance_xxx;
                pcl::PointXYZ sp_yyy;
                sp_yyy.x = length*0.5*0.866*0.67+graspcentre.x;
                sp_yyy.y = length*0.5*0.5*0.67+graspcentre.y;
                sp_yyy.z = 0.3;
                std::vector<int>spindex_yyy;
                std::vector<float>spdistance_yyy;
                if(kdtree.radiusSearch(graspcentre,width_radius,spindex_graspcentre,spdistance_graspcentre)+kdtree.radiusSearch(sp_xx,width_radius,spindex_xx,spdistance_xx)+kdtree.radiusSearch(sp_yy,width_radius,spindex_yy,spdistance_yy)+kdtree.radiusSearch(sp_xxx,width_radius,spindex_xxx,spdistance_xxx)+kdtree.radiusSearch(sp_yyy,width_radius,spindex_yyy,spdistance_yyy))
                {
                weight-= 0.001;
                pcl::PointXYZ point;
                point.x = graspcentre.x;
                point.y = graspcentre.y;
                point.z = 0.13;
                cloud2->points.push_back(point);
                point.z = weight;
                cloud3->points.push_back(point);
                }

            }
            cloud2->width = (uint32_t) cloud2->points.size();
            cloud2->height = 1;
            cloud3->width = (uint32_t) cloud3->points.size();
            cloud3->height = 1;
            //std::cout<< "cloud2.size = "<< cloud2->points.size()<<endl;
            //std::cout<< "weight = "<< weight<<endl;
            //std::cout<< "cloud3.size = "<< cloud3->points.size()<<endl;
            
        }
    }

    //get the location of best three grasp handles
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree2;
    kdtree2.setInputCloud(cloud3);
    float radius2 = 0.015;
    std::vector<int> value_handle;
    int last = 0;
    pcl::PointXYZ bestgrasp1;
    pcl::PointXYZ bestgrasp2;
    pcl::PointXYZ bestgrasp3;
    for(float target_x = min1.x;target_x<max1.x;target_x+=0.005)
    {
        for(float target_y = min1.y;target_y<max1.y;target_y+=0.005)
        {
            pcl::PointXYZ sp_handle;
            sp_handle.x = target_x;
            sp_handle.y = target_y;
            sp_handle.z = 0.1; 
            std::vector<int>spindex_handle;
            std::vector<float>spdistance_handle;
            int value_handle_element = 0;
            if(kdtree2.radiusSearch(sp_handle,radius2,spindex_handle,spdistance_handle) >0 )
            {
                if(value_handle.size()!=0)
                {last = value_handle[value_handle.size()-1];}
                value_handle_element = spindex_handle.size();
                value_handle.push_back(value_handle_element);
                sort(value_handle.begin(),value_handle.end());
                if(last != value_handle.back())
                {
                    bestgrasp1.x = target_x;
                    bestgrasp1.y = target_y;
                }
            }
        }
    }
    std::vector<int> value_handle2;
    int last2 = 0;
    for(float target_x = min1.x;target_x<max1.x;target_x+=0.005)
    {
        for(float target_y = min1.y;target_y<max1.y;target_y+=0.005)
        {
            if(target_x == bestgrasp1.x && target_y == bestgrasp1.y)
            {continue;}
            pcl::PointXYZ sp_handle;
            sp_handle.x = target_x;
            sp_handle.y = target_y;
            sp_handle.z = 0.1; 
            std::vector<int>spindex_handle;
            std::vector<float>spdistance_handle;
            int value_handle_element = 0;
            if(kdtree2.radiusSearch(sp_handle,radius2,spindex_handle,spdistance_handle) >0 )
            {
                if(value_handle2.size()!=0)
                {last2 = value_handle2[value_handle2.size()-1];}
                value_handle_element = spindex_handle.size();
                value_handle2.push_back(value_handle_element);
                sort(value_handle2.begin(),value_handle2.end());
                if(last2 != value_handle2.back())
                {
                    bestgrasp2.x = target_x;
                    bestgrasp2.y = target_y;
                }
            }
        }
    }
    std::vector<int> value_handle3;
    int last3 = 0;
    for(float target_x = min1.x;target_x<max1.x;target_x+=0.005)
    {
        for(float target_y = min1.y;target_y<max1.y;target_y+=0.005)
        {
            if(target_x == bestgrasp1.x && target_y == bestgrasp1.y)
            {continue;}
            if(target_x == bestgrasp2.x && target_y == bestgrasp2.y)
            {continue;}
            pcl::PointXYZ sp_handle;
            sp_handle.x = target_x;
            sp_handle.y = target_y;
            sp_handle.z = 0.1; 
            std::vector<int>spindex_handle;
            std::vector<float>spdistance_handle;
            int value_handle_element = 0;
            if(kdtree2.radiusSearch(sp_handle,radius2,spindex_handle,spdistance_handle) >0 )
            {
                if(value_handle3.size()!=0)
                {last3 = value_handle3[value_handle3.size()-1];}
                value_handle_element = spindex_handle.size();
                value_handle3.push_back(value_handle_element);
                sort(value_handle3.begin(),value_handle3.end());
                if(last3 != value_handle3.back())
                {
                    bestgrasp3.x = target_x;
                    bestgrasp3.y = target_y;
                }
            }
        }
    }
    // the best three grasp handles are (bestgrasp1,bestgrasp2,bestgrasp3)
    std::cout << value_handle[value_handle.size()-1] <<" "<<endl;
    std::cout << value_handle[value_handle.size()-2] <<" "<<endl;
    std::cout << value_handle[value_handle.size()-3] <<" "<<endl;

    //get the best angle of each grasp handle
    float angle = 0;
    std::vector<int> list_angle1;
    for(angle = 0;angle<180;angle+=5)
    {
        int angle_bool = 0;
        float radius3 = finger*0.5;
        pcl::PointXYZ sp_1;
        sp_1.x = (length+finger) * 0.5 *cos(angle*3.1416/180) + bestgrasp1.x ;
        sp_1.y = -(length+finger) * 0.5 *sin(angle*3.1416/180) + bestgrasp1.y;
        sp_1.z = 0.3;
        std::vector<int>spindex_1;
        std::vector<float>spdistance_1;
        pcl::PointXYZ sp_2;
        sp_2.x = -(length+finger) * 0.5 *cos(angle*3.1416/180) +  bestgrasp1.x;
        sp_2.y = (length+finger) * 0.5 *sin(angle*3.1416/180) + bestgrasp1.y;
        sp_2.z = 0.3;
        std::vector<int>spindex_2;
        std::vector<float>spdistance_2;
        if(kdtree.radiusSearch(sp_1,radius3,spindex_1,spdistance_1)+ kdtree.radiusSearch(sp_2,radius3,spindex_2,spdistance_2)==0 )
        {
            angle_bool = 1;
        }
        list_angle1.push_back(angle_bool);
    }
    std::vector<int> rebuild_list_angle1(list_angle1.size());
    if(list_angle1[0])
    rebuild_list_angle1[0] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[list_angle1.size()-6]+list_angle1[list_angle1.size()-5]+list_angle1[list_angle1.size()-4]+list_angle1[list_angle1.size()-3]+list_angle1[list_angle1.size()-2]+list_angle1[list_angle1.size()-1];
    if(list_angle1[1])
    rebuild_list_angle1[1] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[list_angle1.size()-5]+list_angle1[list_angle1.size()-4]+list_angle1[list_angle1.size()-3]+list_angle1[list_angle1.size()-2]+list_angle1[list_angle1.size()-1];
    if(list_angle1[2])
    rebuild_list_angle1[2] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[list_angle1.size()-4]+list_angle1[list_angle1.size()-3]+list_angle1[list_angle1.size()-2]+list_angle1[list_angle1.size()-1];
    if(list_angle1[3])
    rebuild_list_angle1[3] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[list_angle1.size()-3]+list_angle1[list_angle1.size()-2]+list_angle1[list_angle1.size()-1];
    if(list_angle1[4])
    rebuild_list_angle1[4] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[list_angle1.size()-2]+list_angle1[list_angle1.size()-1];
    if(list_angle1[5])
    rebuild_list_angle1[5] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[list_angle1.size()-1];
    if(list_angle1[6])
    rebuild_list_angle1[6] = list_angle1[0]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[7])
    rebuild_list_angle1[7] = list_angle1[13]+list_angle1[1]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[8])
    rebuild_list_angle1[8] = list_angle1[13]+list_angle1[14]+list_angle1[2]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[9])
    rebuild_list_angle1[9] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[10])
    rebuild_list_angle1[10] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[4]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[11])
    rebuild_list_angle1[11] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[5]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[12])
    rebuild_list_angle1[12] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[6]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[13])
    rebuild_list_angle1[13] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[7]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[14])
    rebuild_list_angle1[14] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[8]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[15])
    rebuild_list_angle1[15] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[9]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[16])
    rebuild_list_angle1[16] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[10]+list_angle1[11]+list_angle1[12];
    if(list_angle1[17])
    rebuild_list_angle1[17] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[11]+list_angle1[12];
    if(list_angle1[18])
    rebuild_list_angle1[18] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[12];
    if(list_angle1[19])
    rebuild_list_angle1[19] = list_angle1[13]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[20])
    rebuild_list_angle1[20] = list_angle1[26]+list_angle1[14]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[21])
    rebuild_list_angle1[21] = list_angle1[26]+list_angle1[27]+list_angle1[15]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[22])
    rebuild_list_angle1[22] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[16]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[23])
    rebuild_list_angle1[23] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[17]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[24])
    rebuild_list_angle1[24] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[18]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[25])
    rebuild_list_angle1[25] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[19]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[26])
    rebuild_list_angle1[26] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[20]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[27])
    rebuild_list_angle1[27] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[21]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[28])
    rebuild_list_angle1[28] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[22]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[29])
    rebuild_list_angle1[29] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[23]+list_angle1[24]+list_angle1[25];
    if(list_angle1[30])
    rebuild_list_angle1[30] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[0]+list_angle1[24]+list_angle1[25];
    if(list_angle1[31])
    rebuild_list_angle1[31] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[0]+list_angle1[1]+list_angle1[25];
    if(list_angle1[32])
    rebuild_list_angle1[32] = list_angle1[26]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[0]+list_angle1[1]+list_angle1[2];
    if(list_angle1[33])
    rebuild_list_angle1[33] = list_angle1[3]+list_angle1[27]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[0]+list_angle1[1]+list_angle1[2];
    if(list_angle1[34])
    rebuild_list_angle1[34] = list_angle1[3]+list_angle1[4]+list_angle1[28]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[0]+list_angle1[1]+list_angle1[2];
    if(list_angle1[35])
    rebuild_list_angle1[35] = list_angle1[3]+list_angle1[4]+list_angle1[5]+list_angle1[29]+list_angle1[30]+list_angle1[31]+list_angle1[32]+list_angle1[33]+list_angle1[34]+list_angle1[35]+list_angle1[0]+list_angle1[1]+list_angle1[2];
        

    std::vector<int> rebuild1_list_angle1(list_angle1.size());
    if(rebuild_list_angle1[0])
    rebuild1_list_angle1[0] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[rebuild_list_angle1.size()-6]+rebuild_list_angle1[rebuild_list_angle1.size()-5]+rebuild_list_angle1[rebuild_list_angle1.size()-4]+rebuild_list_angle1[rebuild_list_angle1.size()-3]+rebuild_list_angle1[rebuild_list_angle1.size()-2]+rebuild_list_angle1[rebuild_list_angle1.size()-1];
    if(rebuild_list_angle1[1])
    rebuild1_list_angle1[1] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[rebuild_list_angle1.size()-5]+rebuild_list_angle1[rebuild_list_angle1.size()-4]+rebuild_list_angle1[rebuild_list_angle1.size()-3]+rebuild_list_angle1[rebuild_list_angle1.size()-2]+rebuild_list_angle1[rebuild_list_angle1.size()-1];
    if(rebuild_list_angle1[2])
    rebuild1_list_angle1[2] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[rebuild_list_angle1.size()-4]+rebuild_list_angle1[rebuild_list_angle1.size()-3]+rebuild_list_angle1[rebuild_list_angle1.size()-2]+rebuild_list_angle1[rebuild_list_angle1.size()-1];
    if(rebuild_list_angle1[3])
    rebuild1_list_angle1[3] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[rebuild_list_angle1.size()-3]+rebuild_list_angle1[rebuild_list_angle1.size()-2]+rebuild_list_angle1[rebuild_list_angle1.size()-1];
    if(rebuild_list_angle1[4])
    rebuild1_list_angle1[4] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[rebuild_list_angle1.size()-2]+rebuild_list_angle1[rebuild_list_angle1.size()-1];
    if(rebuild_list_angle1[5])
    rebuild1_list_angle1[5] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[rebuild_list_angle1.size()-1];
    if(rebuild_list_angle1[6])
    rebuild1_list_angle1[6] = rebuild_list_angle1[0]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[12];
    if(rebuild_list_angle1[7])
    rebuild1_list_angle1[7] = rebuild_list_angle1[13]+rebuild_list_angle1[1]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[12];
    if(rebuild_list_angle1[8])
    rebuild1_list_angle1[8] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[2]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[12];
    if(rebuild_list_angle1[9])
    rebuild1_list_angle1[9] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[12];
    if(rebuild_list_angle1[10])
    rebuild1_list_angle1[10] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[12];
    if(rebuild_list_angle1[11])
    rebuild1_list_angle1[11] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[5]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+rebuild_list_angle1[12];
    if(rebuild_list_angle1[12])
    rebuild1_list_angle1[12] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[6]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+list_angle1[12];
    if(rebuild_list_angle1[13])
    rebuild1_list_angle1[13] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[7]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+list_angle1[12];
    if(rebuild_list_angle1[14])
    rebuild1_list_angle1[14] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[8]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+list_angle1[12];
    if(rebuild_list_angle1[15])
    rebuild1_list_angle1[15] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[9]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+list_angle1[12];
    if(rebuild_list_angle1[16])
    rebuild1_list_angle1[16] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[10]+rebuild_list_angle1[11]+list_angle1[12];
    if(rebuild_list_angle1[17])
    rebuild1_list_angle1[17] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[11]+list_angle1[12];
    if(rebuild_list_angle1[18])
    rebuild1_list_angle1[18] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[12];
    if(rebuild_list_angle1[19])
    rebuild1_list_angle1[19] = rebuild_list_angle1[13]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[20])
    rebuild1_list_angle1[20] = rebuild_list_angle1[26]+rebuild_list_angle1[14]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[21])
    rebuild1_list_angle1[21] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[15]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[22])
    rebuild1_list_angle1[22] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[16]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[23])
    rebuild1_list_angle1[23] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[17]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[24])
    rebuild1_list_angle1[24] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[18]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[25])
    rebuild1_list_angle1[25] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[19]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[26])
    rebuild1_list_angle1[26] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[20]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[27])
    rebuild1_list_angle1[27] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[21]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[28])
    rebuild1_list_angle1[28] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[22]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[29])
    rebuild1_list_angle1[29] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[23]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[30])
    rebuild1_list_angle1[30] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[0]+rebuild_list_angle1[24]+list_angle1[25];
    if(rebuild_list_angle1[31])
    rebuild1_list_angle1[31] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[0]+rebuild_list_angle1[1]+list_angle1[25];
    if(rebuild_list_angle1[32])
    rebuild1_list_angle1[32] = rebuild_list_angle1[26]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[0]+rebuild_list_angle1[1]+list_angle1[2];
    if(rebuild_list_angle1[33])
    rebuild1_list_angle1[33] = rebuild_list_angle1[3]+rebuild_list_angle1[27]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[0]+rebuild_list_angle1[1]+list_angle1[2];
    if(rebuild_list_angle1[34])
    rebuild1_list_angle1[34] = rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[28]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[0]+rebuild_list_angle1[1]+list_angle1[2];
    if(rebuild_list_angle1[35])
    rebuild1_list_angle1[35] = rebuild_list_angle1[3]+rebuild_list_angle1[4]+rebuild_list_angle1[5]+rebuild_list_angle1[29]+rebuild_list_angle1[30]+rebuild_list_angle1[31]+rebuild_list_angle1[32]+rebuild_list_angle1[33]+rebuild_list_angle1[34]+rebuild_list_angle1[35]+rebuild_list_angle1[0]+rebuild_list_angle1[1]+list_angle1[2];
    for(int iiiiiii = 0;iiiiiii<list_angle1.size();iiiiiii++)
    {
        std::cout << list_angle1[iiiiiii]<<"   "<<rebuild_list_angle1[iiiiiii]<<" "<<rebuild1_list_angle1[iiiiiii]<< endl;
    }
    std::vector<int>::iterator biggest = std::max_element(rebuild1_list_angle1.begin(),rebuild1_list_angle1.end());
    std::cout << "max is "<< *biggest<<"at " <<std::distance(rebuild1_list_angle1.begin(),biggest)<<endl;
    
    //int npos = (int)(std::max_element(rebuild1_list_angle1.begin(),rebuild1_list_angle1.end())-rebuild1_list_angle1.begin());
    //std::cout << "max is "<< rebuild1_list_angle1[npos] <<"at " <<npos<<endl;
    float angle2 = 0;
    std::vector<int> list_angle2;
    for(angle2 = 0;angle2<180;angle2+=5)
    {
        int angle_bool = 0;
        float radius3 = finger*0.5;
        pcl::PointXYZ sp_1;
        sp_1.x = (length+finger) * 0.5 *cos(angle2*3.1416/180) + bestgrasp2.x ;
        sp_1.y = -(length+finger) * 0.5 *sin(angle2*3.1416/180) + bestgrasp2.y;
        sp_1.z = 0.3;
        std::vector<int>spindex_1;
        std::vector<float>spdistance_1;
        pcl::PointXYZ sp_2;
        sp_2.x = -(length+finger) * 0.5 *cos(angle2*3.1416/180) +  bestgrasp2.x;
        sp_2.y = (length+finger) * 0.5 *sin(angle2*3.1416/180) + bestgrasp2.y;
        sp_2.z = 0.3;
        std::vector<int>spindex_2;
        std::vector<float>spdistance_2;
        if(kdtree.radiusSearch(sp_1,radius3,spindex_1,spdistance_1)+ kdtree.radiusSearch(sp_2,radius3,spindex_2,spdistance_2)==0 )
        {
            angle_bool = 1;
        }
        list_angle2.push_back(angle_bool);
    }
    std::vector<int> rebuild_list_angle2(list_angle2.size());
    if(list_angle2[0])
    rebuild_list_angle2[0] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[list_angle2.size()-6]+list_angle2[list_angle2.size()-5]+list_angle2[list_angle2.size()-4]+list_angle2[list_angle2.size()-3]+list_angle2[list_angle2.size()-2]+list_angle2[list_angle2.size()-1];
    if(list_angle2[1])
    rebuild_list_angle2[1] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[list_angle2.size()-5]+list_angle2[list_angle2.size()-4]+list_angle2[list_angle2.size()-3]+list_angle2[list_angle2.size()-2]+list_angle2[list_angle2.size()-1];
    if(list_angle2[2])
    rebuild_list_angle2[2] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[list_angle2.size()-4]+list_angle2[list_angle2.size()-3]+list_angle2[list_angle2.size()-2]+list_angle2[list_angle2.size()-1];
    if(list_angle2[3])
    rebuild_list_angle2[3] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[list_angle2.size()-3]+list_angle2[list_angle2.size()-2]+list_angle2[list_angle2.size()-1];
    if(list_angle2[4])
    rebuild_list_angle2[4] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[list_angle2.size()-2]+list_angle2[list_angle2.size()-1];
    if(list_angle2[5])
    rebuild_list_angle2[5] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[list_angle2.size()-1];
    if(list_angle2[6])
    rebuild_list_angle2[6] = list_angle2[0]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[7])
    rebuild_list_angle2[7] = list_angle2[13]+list_angle2[1]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[8])
    rebuild_list_angle2[8] = list_angle2[13]+list_angle2[14]+list_angle2[2]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[9])
    rebuild_list_angle2[9] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[10])
    rebuild_list_angle2[10] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[4]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[11])
    rebuild_list_angle2[11] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[5]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[12])
    rebuild_list_angle2[12] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[6]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[13])
    rebuild_list_angle2[13] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[7]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[14])
    rebuild_list_angle2[14] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[8]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[15])
    rebuild_list_angle2[15] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[9]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[16])
    rebuild_list_angle2[16] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[10]+list_angle2[11]+list_angle2[12];
    if(list_angle2[17])
    rebuild_list_angle2[17] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[11]+list_angle2[12];
    if(list_angle2[18])
    rebuild_list_angle2[18] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[12];
    if(list_angle2[19])
    rebuild_list_angle2[19] = list_angle2[13]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[20])
    rebuild_list_angle2[20] = list_angle2[26]+list_angle2[14]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[21])
    rebuild_list_angle2[21] = list_angle2[26]+list_angle2[27]+list_angle2[15]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[22])
    rebuild_list_angle2[22] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[16]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[23])
    rebuild_list_angle2[23] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[17]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[24])
    rebuild_list_angle2[24] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[18]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[25])
    rebuild_list_angle2[25] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[19]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[26])
    rebuild_list_angle2[26] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[20]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[27])
    rebuild_list_angle2[27] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[21]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[28])
    rebuild_list_angle2[28] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[22]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[29])
    rebuild_list_angle2[29] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[23]+list_angle2[24]+list_angle2[25];
    if(list_angle2[30])
    rebuild_list_angle2[30] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[0]+list_angle2[24]+list_angle2[25];
    if(list_angle2[31])
    rebuild_list_angle2[31] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[0]+list_angle2[1]+list_angle2[25];
    if(list_angle2[32])
    rebuild_list_angle2[32] = list_angle2[26]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[0]+list_angle2[1]+list_angle2[2];
    if(list_angle2[33])
    rebuild_list_angle2[33] = list_angle2[3]+list_angle2[27]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[0]+list_angle2[1]+list_angle2[2];
    if(list_angle2[34])
    rebuild_list_angle2[34] = list_angle2[3]+list_angle2[4]+list_angle2[28]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[0]+list_angle2[1]+list_angle2[2];
    if(list_angle2[35])
    rebuild_list_angle2[35] = list_angle2[3]+list_angle2[4]+list_angle2[5]+list_angle2[29]+list_angle2[30]+list_angle2[31]+list_angle2[32]+list_angle2[33]+list_angle2[34]+list_angle2[35]+list_angle2[0]+list_angle2[1]+list_angle2[2];
        

    std::vector<int> rebuild1_list_angle2(list_angle2.size());
    if(rebuild_list_angle2[0])
    rebuild1_list_angle2[0] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[rebuild_list_angle2.size()-6]+rebuild_list_angle2[rebuild_list_angle2.size()-5]+rebuild_list_angle2[rebuild_list_angle2.size()-4]+rebuild_list_angle2[rebuild_list_angle2.size()-3]+rebuild_list_angle2[rebuild_list_angle2.size()-2]+rebuild_list_angle2[rebuild_list_angle2.size()-1];
    if(rebuild_list_angle2[1])
    rebuild1_list_angle2[1] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[rebuild_list_angle2.size()-5]+rebuild_list_angle2[rebuild_list_angle2.size()-4]+rebuild_list_angle2[rebuild_list_angle2.size()-3]+rebuild_list_angle2[rebuild_list_angle2.size()-2]+rebuild_list_angle2[rebuild_list_angle2.size()-1];
    if(rebuild_list_angle2[2])
    rebuild1_list_angle2[2] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[rebuild_list_angle2.size()-4]+rebuild_list_angle2[rebuild_list_angle2.size()-3]+rebuild_list_angle2[rebuild_list_angle2.size()-2]+rebuild_list_angle2[rebuild_list_angle2.size()-1];
    if(rebuild_list_angle2[3])
    rebuild1_list_angle2[3] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[rebuild_list_angle2.size()-3]+rebuild_list_angle2[rebuild_list_angle2.size()-2]+rebuild_list_angle2[rebuild_list_angle2.size()-1];
    if(rebuild_list_angle2[4])
    rebuild1_list_angle2[4] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[rebuild_list_angle2.size()-2]+rebuild_list_angle2[rebuild_list_angle2.size()-1];
    if(rebuild_list_angle2[5])
    rebuild1_list_angle2[5] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[rebuild_list_angle2.size()-1];
    if(rebuild_list_angle2[6])
    rebuild1_list_angle2[6] = rebuild_list_angle2[0]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[12];
    if(rebuild_list_angle2[7])
    rebuild1_list_angle2[7] = rebuild_list_angle2[13]+rebuild_list_angle2[1]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[12];
    if(rebuild_list_angle2[8])
    rebuild1_list_angle2[8] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[2]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[12];
    if(rebuild_list_angle2[9])
    rebuild1_list_angle2[9] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[12];
    if(rebuild_list_angle2[10])
    rebuild1_list_angle2[10] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[12];
    if(rebuild_list_angle2[11])
    rebuild1_list_angle2[11] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[5]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+rebuild_list_angle2[12];
    if(rebuild_list_angle2[12])
    rebuild1_list_angle2[12] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[6]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+list_angle2[12];
    if(rebuild_list_angle2[13])
    rebuild1_list_angle2[13] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[7]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+list_angle2[12];
    if(rebuild_list_angle2[14])
    rebuild1_list_angle2[14] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[8]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+list_angle2[12];
    if(rebuild_list_angle2[15])
    rebuild1_list_angle2[15] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[9]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+list_angle2[12];
    if(rebuild_list_angle2[16])
    rebuild1_list_angle2[16] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[10]+rebuild_list_angle2[11]+list_angle2[12];
    if(rebuild_list_angle2[17])
    rebuild1_list_angle2[17] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[11]+list_angle2[12];
    if(rebuild_list_angle2[18])
    rebuild1_list_angle2[18] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[12];
    if(rebuild_list_angle2[19])
    rebuild1_list_angle2[19] = rebuild_list_angle2[13]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[20])
    rebuild1_list_angle2[20] = rebuild_list_angle2[26]+rebuild_list_angle2[14]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[21])
    rebuild1_list_angle2[21] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[15]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[22])
    rebuild1_list_angle2[22] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[16]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[23])
    rebuild1_list_angle2[23] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[17]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[24])
    rebuild1_list_angle2[24] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[18]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[25])
    rebuild1_list_angle2[25] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[19]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[26])
    rebuild1_list_angle2[26] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[20]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[27])
    rebuild1_list_angle2[27] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[21]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[28])
    rebuild1_list_angle2[28] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[22]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[29])
    rebuild1_list_angle2[29] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[23]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[30])
    rebuild1_list_angle2[30] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[0]+rebuild_list_angle2[24]+list_angle2[25];
    if(rebuild_list_angle2[31])
    rebuild1_list_angle2[31] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[0]+rebuild_list_angle2[1]+list_angle2[25];
    if(rebuild_list_angle2[32])
    rebuild1_list_angle2[32] = rebuild_list_angle2[26]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[0]+rebuild_list_angle2[1]+list_angle2[2];
    if(rebuild_list_angle2[33])
    rebuild1_list_angle2[33] = rebuild_list_angle2[3]+rebuild_list_angle2[27]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[0]+rebuild_list_angle2[1]+list_angle2[2];
    if(rebuild_list_angle2[34])
    rebuild1_list_angle2[34] = rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[28]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[0]+rebuild_list_angle2[1]+list_angle2[2];
    if(rebuild_list_angle2[35])
    rebuild1_list_angle2[35] = rebuild_list_angle2[3]+rebuild_list_angle2[4]+rebuild_list_angle2[5]+rebuild_list_angle2[29]+rebuild_list_angle2[30]+rebuild_list_angle2[31]+rebuild_list_angle2[32]+rebuild_list_angle2[33]+rebuild_list_angle2[34]+rebuild_list_angle2[35]+rebuild_list_angle2[0]+rebuild_list_angle2[1]+list_angle2[2];
    for(int iiiiiii = 0;iiiiiii<list_angle2.size();iiiiiii++)
    {
        std::cout << list_angle2[iiiiiii]<<"   "<<rebuild_list_angle2[iiiiiii]<<" "<<rebuild1_list_angle2[iiiiiii]<< endl;
    }
    std::vector<int>::iterator biggest2 = std::max_element(rebuild1_list_angle2.begin(),rebuild1_list_angle2.end());
    std::cout << "max is "<< *biggest2<<"at " <<std::distance(rebuild1_list_angle2.begin(),biggest2)<<endl;


    float angle3 = 0;
    std::vector<int> list_angle3;
    for(angle3 = 0;angle3<180;angle3+=5)
    {
        int angle_bool = 0;
        float radius3 = finger*0.5;
        pcl::PointXYZ sp_1;
        sp_1.x = (length+finger) * 0.5 *cos(angle3*3.1416/180) + bestgrasp1.x ;
        sp_1.y = -(length+finger) * 0.5 *sin(angle3*3.1416/180) + bestgrasp1.y;
        sp_1.z = 0.3;
        std::vector<int>spindex_1;
        std::vector<float>spdistance_1;
        pcl::PointXYZ sp_2;
        sp_2.x = -(length+finger) * 0.5 *cos(angle3*3.1416/180) +  bestgrasp1.x;
        sp_2.y = (length+finger) * 0.5 *sin(angle3*3.1416/180) + bestgrasp1.y;
        sp_2.z = 0.3;
        std::vector<int>spindex_2;
        std::vector<float>spdistance_2;
        if(kdtree.radiusSearch(sp_1,radius3,spindex_1,spdistance_1)+ kdtree.radiusSearch(sp_2,radius3,spindex_2,spdistance_2)==0 )
        {
            angle_bool = 1;
        }
        list_angle3.push_back(angle_bool);
    }
    std::vector<int> rebuild_list_angle3(list_angle3.size());
    if(list_angle3[0])
    rebuild_list_angle3[0] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[list_angle3.size()-6]+list_angle3[list_angle3.size()-5]+list_angle3[list_angle3.size()-4]+list_angle3[list_angle3.size()-3]+list_angle3[list_angle3.size()-2]+list_angle3[list_angle3.size()-1];
    if(list_angle3[1])
    rebuild_list_angle3[1] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[list_angle3.size()-5]+list_angle3[list_angle3.size()-4]+list_angle3[list_angle3.size()-3]+list_angle3[list_angle3.size()-2]+list_angle3[list_angle3.size()-1];
    if(list_angle3[2])
    rebuild_list_angle3[2] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[list_angle3.size()-4]+list_angle3[list_angle3.size()-3]+list_angle3[list_angle3.size()-2]+list_angle3[list_angle3.size()-1];
    if(list_angle3[3])
    rebuild_list_angle3[3] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[list_angle3.size()-3]+list_angle3[list_angle3.size()-2]+list_angle3[list_angle3.size()-1];
    if(list_angle3[4])
    rebuild_list_angle3[4] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[list_angle3.size()-2]+list_angle3[list_angle3.size()-1];
    if(list_angle3[5])
    rebuild_list_angle3[5] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[list_angle3.size()-1];
    if(list_angle3[6])
    rebuild_list_angle3[6] = list_angle3[0]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[7])
    rebuild_list_angle3[7] = list_angle3[13]+list_angle3[1]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[8])
    rebuild_list_angle3[8] = list_angle3[13]+list_angle3[14]+list_angle3[2]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[9])
    rebuild_list_angle3[9] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[10])
    rebuild_list_angle3[10] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[4]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[11])
    rebuild_list_angle3[11] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[5]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[12])
    rebuild_list_angle3[12] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[6]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[13])
    rebuild_list_angle3[13] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[7]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[14])
    rebuild_list_angle3[14] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[8]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[15])
    rebuild_list_angle3[15] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[9]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[16])
    rebuild_list_angle3[16] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[10]+list_angle3[11]+list_angle3[12];
    if(list_angle3[17])
    rebuild_list_angle3[17] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[11]+list_angle3[12];
    if(list_angle3[18])
    rebuild_list_angle3[18] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[12];
    if(list_angle3[19])
    rebuild_list_angle3[19] = list_angle3[13]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[20])
    rebuild_list_angle3[20] = list_angle3[26]+list_angle3[14]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[21])
    rebuild_list_angle3[21] = list_angle3[26]+list_angle3[27]+list_angle3[15]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[22])
    rebuild_list_angle3[22] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[16]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[23])
    rebuild_list_angle3[23] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[17]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[24])
    rebuild_list_angle3[24] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[18]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[25])
    rebuild_list_angle3[25] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[19]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[26])
    rebuild_list_angle3[26] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[20]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[27])
    rebuild_list_angle3[27] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[21]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[28])
    rebuild_list_angle3[28] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[22]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[29])
    rebuild_list_angle3[29] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[23]+list_angle3[24]+list_angle3[25];
    if(list_angle3[30])
    rebuild_list_angle3[30] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[0]+list_angle3[24]+list_angle3[25];
    if(list_angle3[31])
    rebuild_list_angle3[31] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[0]+list_angle3[1]+list_angle3[25];
    if(list_angle3[32])
    rebuild_list_angle3[32] = list_angle3[26]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[0]+list_angle3[1]+list_angle3[2];
    if(list_angle3[33])
    rebuild_list_angle3[33] = list_angle3[3]+list_angle3[27]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[0]+list_angle3[1]+list_angle3[2];
    if(list_angle3[34])
    rebuild_list_angle3[34] = list_angle3[3]+list_angle3[4]+list_angle3[28]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[0]+list_angle3[1]+list_angle3[2];
    if(list_angle3[35])
    rebuild_list_angle3[35] = list_angle3[3]+list_angle3[4]+list_angle3[5]+list_angle3[29]+list_angle3[30]+list_angle3[31]+list_angle3[32]+list_angle3[33]+list_angle3[34]+list_angle3[35]+list_angle3[0]+list_angle3[1]+list_angle3[2];
        

    std::vector<int> rebuild1_list_angle3(list_angle3.size());
    if(rebuild_list_angle3[0])
    rebuild1_list_angle3[0] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[rebuild_list_angle3.size()-6]+rebuild_list_angle3[rebuild_list_angle3.size()-5]+rebuild_list_angle3[rebuild_list_angle3.size()-4]+rebuild_list_angle3[rebuild_list_angle3.size()-3]+rebuild_list_angle3[rebuild_list_angle3.size()-2]+rebuild_list_angle3[rebuild_list_angle3.size()-1];
    if(rebuild_list_angle3[1])
    rebuild1_list_angle3[1] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[rebuild_list_angle3.size()-5]+rebuild_list_angle3[rebuild_list_angle3.size()-4]+rebuild_list_angle3[rebuild_list_angle3.size()-3]+rebuild_list_angle3[rebuild_list_angle3.size()-2]+rebuild_list_angle3[rebuild_list_angle3.size()-1];
    if(rebuild_list_angle3[2])
    rebuild1_list_angle3[2] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[rebuild_list_angle3.size()-4]+rebuild_list_angle3[rebuild_list_angle3.size()-3]+rebuild_list_angle3[rebuild_list_angle3.size()-2]+rebuild_list_angle3[rebuild_list_angle3.size()-1];
    if(rebuild_list_angle3[3])
    rebuild1_list_angle3[3] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[rebuild_list_angle3.size()-3]+rebuild_list_angle3[rebuild_list_angle3.size()-2]+rebuild_list_angle3[rebuild_list_angle3.size()-1];
    if(rebuild_list_angle3[4])
    rebuild1_list_angle3[4] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[rebuild_list_angle3.size()-2]+rebuild_list_angle3[rebuild_list_angle3.size()-1];
    if(rebuild_list_angle3[5])
    rebuild1_list_angle3[5] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[rebuild_list_angle3.size()-1];
    if(rebuild_list_angle3[6])
    rebuild1_list_angle3[6] = rebuild_list_angle3[0]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[12];
    if(rebuild_list_angle3[7])
    rebuild1_list_angle3[7] = rebuild_list_angle3[13]+rebuild_list_angle3[1]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[12];
    if(rebuild_list_angle3[8])
    rebuild1_list_angle3[8] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[2]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[12];
    if(rebuild_list_angle3[9])
    rebuild1_list_angle3[9] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[12];
    if(rebuild_list_angle3[10])
    rebuild1_list_angle3[10] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[12];
    if(rebuild_list_angle3[11])
    rebuild1_list_angle3[11] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[5]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+rebuild_list_angle3[12];
    if(rebuild_list_angle3[12])
    rebuild1_list_angle3[12] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[6]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+list_angle3[12];
    if(rebuild_list_angle3[13])
    rebuild1_list_angle3[13] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[7]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+list_angle3[12];
    if(rebuild_list_angle3[14])
    rebuild1_list_angle3[14] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[8]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+list_angle3[12];
    if(rebuild_list_angle3[15])
    rebuild1_list_angle3[15] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[9]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+list_angle3[12];
    if(rebuild_list_angle3[16])
    rebuild1_list_angle3[16] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[10]+rebuild_list_angle3[11]+list_angle3[12];
    if(rebuild_list_angle3[17])
    rebuild1_list_angle3[17] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[11]+list_angle3[12];
    if(rebuild_list_angle3[18])
    rebuild1_list_angle3[18] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[12];
    if(rebuild_list_angle3[19])
    rebuild1_list_angle3[19] = rebuild_list_angle3[13]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[20])
    rebuild1_list_angle3[20] = rebuild_list_angle3[26]+rebuild_list_angle3[14]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[21])
    rebuild1_list_angle3[21] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[15]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[22])
    rebuild1_list_angle3[22] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[16]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[23])
    rebuild1_list_angle3[23] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[17]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[24])
    rebuild1_list_angle3[24] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[18]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[25])
    rebuild1_list_angle3[25] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[19]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[26])
    rebuild1_list_angle3[26] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[20]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[27])
    rebuild1_list_angle3[27] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[21]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[28])
    rebuild1_list_angle3[28] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[22]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[29])
    rebuild1_list_angle3[29] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[23]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[30])
    rebuild1_list_angle3[30] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[0]+rebuild_list_angle3[24]+list_angle3[25];
    if(rebuild_list_angle3[31])
    rebuild1_list_angle3[31] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[0]+rebuild_list_angle3[1]+list_angle3[25];
    if(rebuild_list_angle3[32])
    rebuild1_list_angle3[32] = rebuild_list_angle3[26]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[0]+rebuild_list_angle3[1]+list_angle3[2];
    if(rebuild_list_angle3[33])
    rebuild1_list_angle3[33] = rebuild_list_angle3[3]+rebuild_list_angle3[27]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[0]+rebuild_list_angle3[1]+list_angle3[2];
    if(rebuild_list_angle3[34])
    rebuild1_list_angle3[34] = rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[28]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[0]+rebuild_list_angle3[1]+list_angle3[2];
    if(rebuild_list_angle3[35])
    rebuild1_list_angle3[35] = rebuild_list_angle3[3]+rebuild_list_angle3[4]+rebuild_list_angle3[5]+rebuild_list_angle3[29]+rebuild_list_angle3[30]+rebuild_list_angle3[31]+rebuild_list_angle3[32]+rebuild_list_angle3[33]+rebuild_list_angle3[34]+rebuild_list_angle3[35]+rebuild_list_angle3[0]+rebuild_list_angle3[1]+list_angle3[2];
    for(int iiiiiii = 0;iiiiiii<list_angle3.size();iiiiiii++)
    {
        std::cout << list_angle3[iiiiiii]<<"   "<<rebuild_list_angle3[iiiiiii]<<" "<<rebuild1_list_angle3[iiiiiii]<< endl;
    }
    std::vector<int>::iterator biggest3 = std::max_element(rebuild1_list_angle3.begin(),rebuild1_list_angle3.end());
    std::cout << "max is "<< *biggest3<<"at " <<std::distance(rebuild1_list_angle3.begin(),biggest3)<<endl;

    int best_angle_index1 = std::distance(rebuild1_list_angle1.begin(),biggest);
    int best_angle_index2 = std::distance(rebuild1_list_angle2.begin(),biggest2);
    int best_angle_index3 = std::distance(rebuild1_list_angle3.begin(),biggest3);
    std::cout << "best1 is "<<best_angle_index1<<" best2 is "<<best_angle_index2<<" best3 is "<<best_angle_index3<<endl;

    // put out
    bestgrasp1.z = min.z;
    bestgrasp2.z = min.z;
    bestgrasp3.z = min.z;
    cloud_grasp->points.push_back(bestgrasp1);
    cloud_grasp->points.push_back(bestgrasp2);
    cloud_grasp->points.push_back(bestgrasp3);
    cloud_grasp->width = cloud_grasp->points.size();
    cloud_grasp->height = 1;
    angle_grasp.push_back(best_angle_index1);
    angle_grasp.push_back(best_angle_index2);
    angle_grasp.push_back(best_angle_index3);
    }
    
    for(int indexx = 0;indexx<angle_grasp.size();indexx++) 
    {
        std::cout << "angle_grasp is "<<angle_grasp[indexx]<<endl;
    }
    //display grasp handles
    
    pcl::PointXYZ p1;
    p1.x = cloud_grasp->points[0].x;
    p1.y = cloud_grasp->points[0].y;
    p1.z = 0;
    
    pcl::PointXYZ p2;
    p2.x = cloud_grasp->points[1].x;
    p2.y = cloud_grasp->points[1].y;
    p2.z = 0;
    
    pcl::PointXYZ p3;   
    p3.x = cloud_grasp->points[2].x;
    p3.y = cloud_grasp->points[2].y;
    p3.z = 0;
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_display(new pcl::PointCloud<pcl::PointXYZ>());
    for(int ddddd = 0;ddddd<cloud_grasp->size();ddddd++)
    {
        //cloud_display->points[ddddd] = cloud_grasp->points[ddddd];
        //cloud_display->points[ddddd].z = cloud_grasp->points[ddddd].z - 0.1;
        //cloud_display->points.push_back(cloud_grasp->points[ddddd]);
        //cloud_display->points[ddddd].z -=0.1;
        pcl::PointXYZ point;
        point.x = cloud_grasp->points[ddddd].x;
        point.y = cloud_grasp->points[ddddd].y;
        point.z = cloud_grasp->points[ddddd].z - 0.1;
        cloud_display->points.push_back(point);
        std::stringstream ss;
        ss << "handle" << ddddd;
        viewer->addArrow<pcl::PointXYZ>(cloud_grasp->points[ddddd],point,0,255,0,ss.str());
        pcl::PointXYZ point_a;
        point_a.x = (length+finger)*0.5*cos(angle_grasp[ddddd]*5*3.1416/180)+cloud_grasp->points[ddddd].x;
        point_a.y = -(length+finger)*0.5*sin(angle_grasp[ddddd]*5*3.1416/180)+cloud_grasp->points[ddddd].y;
        point_a.z = cloud_grasp->points[ddddd].z;
        pcl::PointXYZ point_b;
        point_b.x = -(length+finger)*0.5*cos(angle_grasp[ddddd]*5*3.1416/180)+cloud_grasp->points[ddddd].x;
        point_b.y = (length+finger)*0.5*sin(angle_grasp[ddddd]*5*3.1416/180)+cloud_grasp->points[ddddd].y;
        point_b.z = cloud_grasp->points[ddddd].z;
        std::stringstream ss2;
        ss2 << "hand" << ddddd;
        viewer->addLine<pcl::PointXYZ>(point_a,point_b,0,255,0,ss2.str());
        pcl::PointXYZ point_c;
        point_c.x = point_a.x;
        point_c.y = point_a.y;
        point_c.z = cloud_grasp->points[ddddd].z + 0.04;
        std::stringstream ss3;
        ss3 << "finger_r" << ddddd;
        viewer->addLine<pcl::PointXYZ>(point_a,point_c,0,255,0,ss3.str());
        pcl::PointXYZ point_d;
        point_d.x = point_b.x;
        point_d.y = point_b.y;
        point_d.z = cloud_grasp->points[ddddd].z + 0.04;
        std::stringstream ss4;
        ss4 << "finger_l" << ddddd;
        viewer->addLine<pcl::PointXYZ>(point_b,point_d,0,255,0,ss4.str());
        
    }
    cloud_display->width = cloud_display->points.size();
    cloud_display->height = 1;
    std::cout << "cloud_display.size() is  "<<cloud_display->size()<<endl;


    cloud_mutex.lock();    // for not overwriting the point cloud

    // Display pointcloud:


  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>("/home/zsm/catkin_ws/src/rospcl/src/rgbtest222.pcd", *cloud_rgba))
    {
        std::cerr << "ERROR: Cannot open file "  << "! Aborting..." << std::endl;
      
    }

    viewer->addPointCloud(cloud_rgba, "xxx");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud, "bunny");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud1, "bunny1");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud2, "bunny2");
    //viewer->addPointCloud<pcl::PointXYZ> (cloud3, "bunny3");
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    cloud_mutex.unlock();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}