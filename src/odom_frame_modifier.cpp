#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

// 名前空間に基づいてframe_idを修正し、新しいトピックにリパブリッシュするノード
class OdomFrameModifier {
public:
    OdomFrameModifier(ros::NodeHandle& nh, const std::string& input_topic, const std::string& output_topic, const std::string& frame_id_prefix) {
        // サブスクライバー
        odom_sub_ = nh.subscribe(input_topic, 10, &OdomFrameModifier::odomCallback, this);
        // パブリッシャー（変更後のトピックに出力）
        odom_pub_ = nh.advertise<nav_msgs::Odometry>(output_topic, 10);
        // frame_idの接頭辞を設定
        frame_id_prefix_ = frame_id_prefix;
    }

private:
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    std::string frame_id_prefix_;

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // メッセージをコピー
        nav_msgs::Odometry modified_msg = *msg;
        // frame_idを修正
        modified_msg.header.frame_id = frame_id_prefix_ + "/odom";
        modified_msg.child_frame_id = frame_id_prefix_ + "/base_link"; // 必要に応じて修正
        // 修正したメッセージを新しいトピックにパブリッシュ
        odom_pub_.publish(modified_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_frame_modifier");
    ros::NodeHandle nh;

    // 名前空間やトピック名、frame_idの接頭辞を設定
    std::string ns = ros::this_node::getNamespace();
    std::string input_topic = ns + "/odom";              // 入力トピック（元のトピック）
    std::string output_topic = ns + "/odom_modified";    // 出力トピック（変更後）
    std::string frame_id_prefix = ns.substr(1);          // '/'を削除した名前空間を接頭辞に

    // ノードをインスタンス化
    OdomFrameModifier odom_modifier(nh, input_topic, output_topic, frame_id_prefix);

    ros::spin();
    return 0;
}
