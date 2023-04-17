#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <ros/ros.h>

nav_msgs::OccupancyGrid cost_map_1;
nav_msgs::OccupancyGrid cost_map_2;
nav_msgs::OccupancyGrid merged_map;

bool cost_map_1_updated = 0;
bool cost_map_2_updated = 0;

// bool cost_map_1_init = 0;
// bool cost_map_2_init = 0;

void cost_map_1_save(nav_msgs::OccupancyGrid cost_map){
    cost_map_1 = cost_map;
    // cost_map_1.data = cost_map.data;
    // cost_map_1.info = cost_map.info;
    // cost_map_1_init = 1;
    cost_map_1_updated = 1;
}

void cost_map_2_save(nav_msgs::OccupancyGrid cost_map){
    cost_map_2 = cost_map;
    // cost_map_2.data = cost_map.data;
    // cost_map_2.info = cost_map.info;
    // cost_map_2_init = 1;
    cost_map_2_updated = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_stack");
    ros::NodeHandle n;
    ros::Publisher merged_map_pub = n.advertise<nav_msgs::OccupancyGrid>("/merged_map", 1);
    ros::Subscriber cost_map_1_sub = n.subscribe("/Quadrotor_1/projected_map", 1, cost_map_1_save);
    ros::Subscriber cost_map_2_sub = n.subscribe("/Quadrotor_2/projected_map", 1, cost_map_2_save);
    ros::Rate loop_rate(5);
    ros::Time start(ros::Time::now());

    merged_map.header.frame_id = "world";
    merged_map.info.resolution = 0.5;
    merged_map.info.origin.orientation.w = 1.0;
    
    while (ros::ok()) {
        // ROS_DEBUG("stack");
        if(cost_map_1_updated && cost_map_2_updated){
            merged_map.data.clear();
            merged_map.info.origin.position.x = cost_map_1.info.origin.position.x <= cost_map_2.info.origin.position.x ?
                             cost_map_1.info.origin.position.x : cost_map_2.info.origin.position.x;

            merged_map.info.origin.position.y = cost_map_1.info.origin.position.y <= cost_map_2.info.origin.position.y ?
                             cost_map_1.info.origin.position.y : cost_map_2.info.origin.position.y;

            float top_left_x =  cost_map_1.info.origin.position.x+cost_map_1.info.resolution*cost_map_1.info.width >= cost_map_2.info.origin.position.x+cost_map_2.info.resolution*cost_map_2.info.width ?
                                cost_map_1.info.origin.position.x+cost_map_1.info.resolution*cost_map_1.info.width : cost_map_2.info.origin.position.x+cost_map_2.info.resolution*cost_map_2.info.width;
            
            float top_left_y =  cost_map_1.info.origin.position.y+cost_map_1.info.resolution*cost_map_1.info.height >= cost_map_2.info.origin.position.y+cost_map_2.info.resolution*cost_map_2.info.height ?
                                cost_map_1.info.origin.position.y+cost_map_1.info.resolution*cost_map_1.info.height : cost_map_2.info.origin.position.y+cost_map_2.info.resolution*cost_map_2.info.height;
            
            merged_map.info.width = (top_left_x - merged_map.info.origin.position.x) / merged_map.info.resolution;
            merged_map.info.height = (top_left_y - merged_map.info.origin.position.y) / merged_map.info.resolution;

            // std_msgs::Int8 
            // data[merged_map.info.width*merged_map.info.height] = {-1};
            int start_1_x = (cost_map_1.info.origin.position.x - merged_map.info.origin.position.x)/merged_map.info.resolution;
            int start_1_y = (cost_map_1.info.origin.position.y - merged_map.info.origin.position.y)/merged_map.info.resolution;

            int start_2_x = (cost_map_2.info.origin.position.x - merged_map.info.origin.position.x)/merged_map.info.resolution;
            int start_2_y = (cost_map_2.info.origin.position.y - merged_map.info.origin.position.y)/merged_map.info.resolution;

            for(int y=0; y<merged_map.info.height; y++){
                for(int x=0; x<merged_map.info.width; x++){
                    merged_map.data.push_back(-1);
                }
            }

            for(int y=0; y<cost_map_1.info.height; y++){
                for(int x=0; x<cost_map_1.info.width; x++){
                    merged_map.data[start_1_x+(start_1_y+y)*merged_map.info.width+x] = cost_map_1.data[x + y*cost_map_1.info.width];
                }
            }

            for(int y=0; y<cost_map_2.info.height; y++){
                for(int x=0; x<cost_map_2.info.width; x++){
                    int index = x + y*cost_map_2.info.width;
                    if(cost_map_2.data[index]>merged_map.data[start_2_x+(start_2_y+y)*merged_map.info.width+x]){
                        merged_map.data[start_2_x+(start_2_y+y)*merged_map.info.width+x] = cost_map_2.data[index];
                    }
                }
            }
            ros::Time now = ros::Time::now();
            merged_map.info.map_load_time = now;
            merged_map.header.stamp = now;

            cost_map_1_updated=0;
            cost_map_2_updated=0;

            merged_map_pub.publish(merged_map);

        }
        
        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}