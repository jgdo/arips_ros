#include <memory>
#include <thread>

#include <ros/ros.h>
#include <ros/master.h>

#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <arips_arm_msgs/ParameterValue.h>
#include <arips_arm_msgs/GetParameter.h>

static const std::string clientParamUpdateTopic = "/tr_param_updates"; // needs preceeding /

bool set_parameters(dynamic_reconfigure::Reconfigure::Request  &req,
         dynamic_reconfigure::Reconfigure::Response &res)
{
    res.config = req.config;
    return true;
}

class ParameterProxy {
public:
    enum State {
        WAITING_PARAMS,
        INITIALIZED,
        FINISHED,
    };

    ParameterProxy(const std::string& ns):
            mNamespace(ns) {
        mClientParameterValueSub = mNh.subscribe(ns + clientParamUpdateTopic, 10, &ParameterProxy::clientParameterValuesCb, this);
        mClientGetParamsClient = mNh.serviceClient<arips_arm_msgs::GetParameter>(ns + "/tr_get_param");
    }

    ParameterProxy(const ParameterProxy&) = delete;

    void startConfigure() {
        if(mState != WAITING_PARAMS) {
            return;
        }

        mInitThread = std::make_unique<std::thread>(&ParameterProxy::waitAndConfigClient, this);
    }

private:
    struct ParamEntry {
        std::string name;
    };

    State mState = WAITING_PARAMS;

    std::unique_ptr<std::thread> mInitThread;

    ros::NodeHandle mNh;
    std::string mNamespace;

    ros::ServiceServer mSetParamsService;

    ros::Publisher mConfigDescriptionPub;
    ros::Publisher mConfigUpdatePub;

    ros::Subscriber mClientParameterValueSub;
    ros::ServiceClient mClientGetParamsClient;

    std::vector<ParamEntry> mParameterList;

    void waitAndConfigClient() {
        if(mState != WAITING_PARAMS) {
            return;
        }

        arips_arm_msgs::GetParameter getParam;

        // get first param
        ros::Rate r(0.5);
        while(mParameterList.empty()) {
            getParam.request.parameter_id = 0;
            if(mClientGetParamsClient.call(getParam)) {
                if(getParam.response.num_total_parameters > 0) {
                    mParameterList.resize(getParam.response.num_total_parameters);
                    configParameter(getParam.response);

                    ROS_INFO_STREAM("Client '" << mNamespace << "' has " << getParam.response.num_total_parameters << " parameters");
                } else {
                    ROS_WARN_STREAM("Parameter client '" << mNamespace << "' reported 0 parameters. Ignoring.");
                    mState = FINISHED;
                    break;
                }
            } else {
                ROS_WARN_STREAM("Failed to call GetParameter service '" << mClientGetParamsClient.getService() << "', retrying...");
                r.sleep();
            }
        }

        for(size_t i = 1; i < mParameterList.size(); i++) {
            while(true) {
                getParam.request.parameter_id = i;
                if (mClientGetParamsClient.call(getParam)) {
                    // TODO: check if num parameters changed
                    configParameter(getParam.response);
                    break;
                } else {
                    ROS_WARN_STREAM("Failed to call GetParameter service '" << mClientGetParamsClient.getService() << "', retrying...");
                    r.sleep();
                }
            }
        }

        configureClient();
    }

    void configureClient() {
        if(mState != WAITING_PARAMS) {
            return;
        }

        mSetParamsService = mNh.advertiseService(mNamespace + "/set_parameters", set_parameters);
        mConfigDescriptionPub = mNh.advertise<dynamic_reconfigure::ConfigDescription>(mNamespace + "/parameter_descriptions", 1, true);
        mConfigUpdatePub = mNh.advertise<dynamic_reconfigure::Config>(mNamespace + "/parameter_updates", 1, true);

        dynamic_reconfigure::ConfigDescription cd;
        cd.groups.resize(1);
        cd.groups.at(0).name = "Default";
        cd.groups.at(0).type = "";
        cd.groups.at(0).id = 0;
        cd.groups.at(0).parent = 0;
        cd.groups.at(0).parameters.resize(2);
        cd.groups.at(0).parameters.at(0).name = "int_param";
        cd.groups.at(0).parameters.at(0).type = "int";
        cd.groups.at(0).parameters.at(0).level = 0;
        cd.groups.at(0).parameters.at(0).description = "An Integer parameter";
        cd.groups.at(0).parameters.at(0).edit_method = "";
        cd.groups.at(0).parameters.at(1).name = "double_param";
        cd.groups.at(0).parameters.at(1).type = "double";
        cd.groups.at(0).parameters.at(1).level = 0;
        cd.groups.at(0).parameters.at(1).description = "An douple parameter";
        cd.groups.at(0).parameters.at(1).edit_method = "";

        cd.max.ints.resize(1);
        cd.max.ints.at(0).name = "int_param";
        cd.max.ints.at(0).value = 100;

        cd.min.ints.resize(1);
        cd.min.ints.at(0).name = "int_param";
        cd.min.ints.at(0).value = 0;

        cd.dflt.ints.resize(1);
        cd.dflt.ints.at(0).name = "int_param";
        cd.dflt.ints.at(0).value = 42;

        cd.max.doubles.resize(1);
        cd.max.doubles.at(0).name = "double_param";
        cd.max.doubles.at(0).value = 1.0;

        cd.min.doubles.resize(1);
        cd.min.doubles.at(0).name = "double_param";
        cd.min.doubles.at(0).value = 0.0;

        cd.dflt.doubles.resize(1);
        cd.dflt.doubles.at(0).name = "double_param";
        cd.dflt.doubles.at(0).value = 0.5;

        cd.max.groups.resize(1);
        cd.max.groups.at(0).name = "Default";
        cd.max.groups.at(0).state = true;
        cd.max.groups.at(0).id = 0;
        cd.max.groups.at(0).parent = 0;

        cd.min.groups.resize(1);
        cd.min.groups.at(0).name = "Default";
        cd.min.groups.at(0).state = true;
        cd.min.groups.at(0).id = 0;
        cd.min.groups.at(0).parent = 0;

        cd.dflt.groups.resize(1);
        cd.dflt.groups.at(0).name = "Default";
        cd.dflt.groups.at(0).state = true;
        cd.dflt.groups.at(0).id = 0;
        cd.dflt.groups.at(0).parent = 0;

        mConfigDescriptionPub.publish(cd);

        dynamic_reconfigure::Config config;
        config.ints.resize(1);
        config.ints.at(0).name = "int_param";
        config.ints.at(0).value = 42;

        config.doubles.resize(1);
        config.doubles.at(0).name = "double_param";
        config.doubles.at(0).value = 0.5;

        config.groups.resize(1);
        config.groups.at(0).name = "Default";
        config.groups.at(0).state = true;
        config.groups.at(0).id = 0;
        config.groups.at(0).parent = 0;

        mConfigUpdatePub.publish(config);

        mState = INITIALIZED;

        ROS_INFO_STREAM("Configured client '" << mNamespace << "'");
    }

    void configParameter(arips_arm_msgs::GetParameter::Response const& res)  {
        mParameterList.at(res.value.id).name = res.name;
    }

    void clientParameterValuesCb(const arips_arm_msgs::ParameterValue& param) {

    }
};

inline bool ends_with(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "parameter_proxy");
    ros::NodeHandle nh;

    std::map<std::string, ParameterProxy> allProxies;

    ros::Rate r(0.5);
    while (ros::ok())
    {
        ros::master::V_TopicInfo topic_infos;
        ros::master::getTopics(topic_infos);

        for(auto const& info: topic_infos) {
            if(info.datatype == "arips_arm_msgs/ParameterValue" && ends_with(info.name, clientParamUpdateTopic)) {
                std::string ns = info.name.substr(0, info.name.size() - clientParamUpdateTopic.size());

                if(!ns.empty()) {
                    if(allProxies.find(ns) == allProxies.end()) {
                        allProxies.emplace(std::piecewise_construct, std::forward_as_tuple(ns), forward_as_tuple(ns)).first->second.startConfigure();
                        ROS_INFO_STREAM("Discovered new parameter client '" << ns << "'");
                    }
                } else {
                    ROS_WARN_STREAM("Client param update topic '" << info.name << "' has empty namespace. Ignoring.");
                }
            }
        }

        ros::spinOnce();
        r.sleep();
    }
}
