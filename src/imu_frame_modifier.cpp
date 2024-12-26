#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

class ImuFrameModifier {
public:
    ImuFrameModifier(ros::NodeHandle& nh, const std::string& input_topic, const std::string& output_topic, const std::string& frame_id_prefix) {
        // サブスクライバー
        imu_sub_ = nh.subscribe(input_topic, 10, &ImuFrameModifier::imuCallback, this);
        // パブリッシャー（変更後のトピックに出力）
        imu_pub_ = nh.advertise<sensor_msgs::Imu>(output_topic, 10);
        // frame_idの接頭辞を設定
        frame_id_prefix_ = frame_id_prefix;
    }

private:
    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;
    std::string frame_id_prefix_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // メッセージをコピー
        sensor_msgs::Imu modified_msg = *msg;
        // frame_idを修正
        modified_msg.header.frame_id = frame_id_prefix_ + "/imu_link";
        // 修正したメッセージを新しいトピックにパブリッシュ
        imu_pub_.publish(modified_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_frame_modifier");
    ros::NodeHandle nh;

    // 名前空間やトピック名、frame_idの接頭辞を設定
    std::string ns = ros::this_node::getNamespace();
    std::string input_topic = ns + "/imu";              // 入力トピック（元のトピック）
    std::string output_topic = ns + "/imu_modified";    // 出力トピック（変更後）
    std::string frame_id_prefix = ns.substr(1);         // '/'を削除した名前空間を接頭辞に

    // ノードをインスタンス化
    ImuFrameModifier imu_modifier(nh, input_topic, output_topic, frame_id_prefix);

    ros::spin();
    return 0;
}
