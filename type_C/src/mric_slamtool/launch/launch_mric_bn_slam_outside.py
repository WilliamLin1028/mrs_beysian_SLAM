from launch import LaunchDescription
from launch_ros.actions import Node

'''
程式架構 :
[database - ]       
    ┌─────────────────────────────────┐
    │                                 ┼─────────rq────────>[outside_check_auctioneer_agent]
    │ outside_auctioneer_agent - ph 2 ┼<────────Y/N───────────────────────┘
    │                                 ┼<──────── not exist ────────────┐
    └───────┼───────────────┼─────────┘                                │
            Y               └── N ─────────────────────> cmd_wait_for_auctioneer.txt
    ┌───────┼─────────────────────────┐
    │ outside_auctioneer_agent - ph 3 ┼───────────────────>[outside_auc_lock_reciver]
    │                                 ┼<──────── feedback ────────────┘
    │                                 ┼───────────────────>[outside_task_agent]<───── /{robot_name}_merge_map
    │       (update tasks list)       ┼<───── new task list ─────────┘                 local_successrate.txt ─────┐
    └─────────┼───────────────────────┘                                                            ┌──────────────┼──────────────┐
    ┌─────────┼───────────────────────┐                ┌──────────────────────────┐                │ outside_sr_rq_reciver(rb_A) │
    │ outside_auctioneer_agent - ph 4 ┼── task list ──>┼                          ┼───── rq ──────>┼             ...             │
    │       (request successrate)     ┼<───────┐       │                          ┼<── rb_sr_list──┼ outside_sr_rq_reciver(rb_E) │
    └─────────────────────────────────┘        │       │ outside_allocation_agent │                └─────────────────────────────┘ 
                                           feedback    │                          │                  ┌───────────────────────────────┐
                                               │       │                          ┼─ ongoing tast───>┼ outside_newtask_reciver(rb_A) │
                                               └───────┼                          ┼<─── feedback ────┼              ...              │
                                                       └──────────────────────────┘                  │ outside_newtask_reciver(rb_E) │
                                                                                                     └───────────────────────────────┘
                    
    ┌──────────────────────────┐
    │ outside_merge_map_agent  ┼<────── /robot_A_map, /robot_B_map ... /robot_E_map
    │ outside_merge_map_agent  ┼──────> /{robot_name}_merge_map
    └──────────────────────────┘
'''

def generate_launch_description():
    ld = LaunchDescription()

    experiment_trigger_agent = Node(
        package='mric_slamtool', 
        executable='outside_trigger_experiment',  
    )

    auctioneer_agent = Node(
        package='mric_slamtool',  
        executable='outside_auctioneer_agent',  
    )
    
    save_merge_map = Node(
        package='mric_slamtool',  
        executable='outside_save_merge_map', 
    )
    
    merge_map_agent = Node(
        package='mric_slamtool',  
        executable='outside_merge_map_agent',
    )
    
    pub_local_map = Node(
        package='mric_slamtool',  
        executable='outside_pub_local_map',
    )
    
    pub_workstatus = Node(
        package='mric_slamtool', 
        executable='outside_pub_workstatus',  
    )
    
    outside_allocation_agent = Node(
        package='mric_slamtool', 
        executable='outside_allocation_agent',  
    )
    

    outside_check_auctioneer_agent = Node(
        package='mric_slamtool', 
        executable='outside_check_auctioneer_agent',  
    )
    
    
    outside_task_agent = Node(
        package='mric_slamtool', 
        executable='outside_task_agent',  
    )
    
    outside_sr_rq_reciver = Node(
        package='mric_slamtool', 
        executable='outside_sr_rq_reciver',  
    )
    
    
    outside_newtask_reciver = Node(
        package='mric_slamtool', 
        executable='outside_newtask_reciver',  
    )
    
    outside_auc_lock_reciver = Node(
        package='mric_slamtool', 
        executable='outside_auc_lock_reciver',  
    )


    ld.add_action(experiment_trigger_agent)
    #ld.add_action(save_merge_map)
    ld.add_action(merge_map_agent)
    #ld.add_action(pub_local_map)
    ld.add_action(pub_workstatus)
    ld.add_action(outside_newtask_reciver)
    ld.add_action(outside_check_auctioneer_agent)
    ld.add_action(outside_task_agent)
    ld.add_action(outside_sr_rq_reciver)
    ld.add_action(outside_allocation_agent)
    ld.add_action(outside_auc_lock_reciver)
    ld.add_action(auctioneer_agent)
    

    return ld
