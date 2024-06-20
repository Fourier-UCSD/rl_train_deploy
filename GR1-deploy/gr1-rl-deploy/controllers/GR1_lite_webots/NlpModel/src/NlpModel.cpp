#include "../include/NlpModel.h"
#include "../../RobotController/include/basicfunction.h"
#define TIMEMEASURE
#ifdef TIMEMEASURE
#include "../../example/include/broccoli/core/Time.hpp"
using namespace broccoli::core;
#endif
NlpModel::NlpModel() {}

NlpModel::~NlpModel() {}

void NlpModel::init(std::string model_pb_path, std::string encoder_pb_path, int obs_num_, int encoder_num_, int action_num_, int latent_num_, int lstm_layers_, int lstm_hidden_size_) {
    // init para
    obs_num = obs_num_;
    encoder_num = encoder_num_;
    action_num = action_num_;
    latent_num = latent_num_;
    lstm_layers = lstm_layers_;
    lstm_hidden_size = lstm_hidden_size_;

    inputdata_nlp = Eigen::VectorXd::Zero(obs_num + 2 * lstm_layers * lstm_hidden_size);
    inputdata_encoder_nlp = Eigen::VectorXd::Zero(encoder_num + 2 * lstm_layers * lstm_hidden_size);
    outputdata_nlp = Eigen::VectorXd::Zero(action_num + 2 * lstm_layers * lstm_hidden_size);
    outputdata_encoder_nlp = Eigen::VectorXd::Zero(latent_num + 2 * lstm_layers * lstm_hidden_size);
    // outputdata_nlp = Eigen::VectorXd::Zero(action_num + 2 * lstm_layers * lstm_hidden_size);
    outputdata_nlp_filter = Eigen::VectorXd::Zero(action_num);
    compute_complete = true;
    // load pt
    std::cout << "load nlp model: " << model_pb_path << std::endl;
    nlpmodel_ = torch::jit::load(model_pb_path, torch::kCPU);
    encodermodel_ = torch::jit::load(encoder_pb_path, torch::kCPU);
    std::cout << "nlp model load complete! " << model_pb_path << std::endl;
    //
    // nlpmodel_thread = std::thread(&NlpModel::act_interface, this);
}

Eigen::VectorXd NlpModel::act_interface_encoder(Eigen::VectorXd inputdata_encoder_nlp) {
    torch::Device device(torch::kCPU);

    std::vector<torch::jit::IValue> inputs_encoder;
    // torch::Tensor inputdata_encoder = torch::zeros({encoder_num + 2 * lstm_layers * lstm_hidden_size}).toType(torch::kFloat);
    torch::Tensor inputdata_encoder = torch::zeros({1, encoder_num + 2 * lstm_layers * lstm_hidden_size}).toType(torch::kFloat);
    basicfunction::copytoinputs(inputdata_encoder, inputdata_encoder_nlp, encoder_num + 2 * lstm_layers * lstm_hidden_size);
    inputdata_encoder.to(device);
    // inputs_encoder.push_back(inputdata_encoder);
    inputs_encoder.push_back(inputdata_encoder.unsqueeze(0));
    torch::Tensor outputdata_encoder = encodermodel_.forward(inputs_encoder).toTensor();
    std::vector<float> v_encoder(outputdata_encoder.data_ptr<float>(), outputdata_encoder.data_ptr<float>() + outputdata_encoder.numel());
    for (int i = 0; i < latent_num + 2 * lstm_layers * lstm_hidden_size; i++) {
        outputdata_encoder_nlp(i) = v_encoder[i];
    }
    return outputdata_encoder_nlp;
}
void NlpModel::act_interface() {

    torch::Device device(torch::kCPU);

    // std::cout<<"act interface run!"<<std::endl;
    std::vector<torch::jit::IValue> inputs;
    torch::Tensor inputdata = torch::zeros({obs_num + 2 * lstm_layers * lstm_hidden_size}).toType(torch::kFloat);

    basicfunction::copytoinputs(inputdata, inputdata_nlp, obs_num + 2 * lstm_layers * lstm_hidden_size);
    inputdata.to(device);
    inputs.push_back(inputdata);
    // std::cout<<"act interface run here!"<<std::endl;
    // std::cout<<"inputdata num : "<<obs_num + 2*lstm_layers*lstm_hidden_size<<std::endl;

    torch::Tensor outputdata = nlpmodel_.forward(inputs).toTensor();
    // std::cout<<"act interface complete!"<<std::endl;
    std::vector<float> v(outputdata.data_ptr<float>(), outputdata.data_ptr<float>() + outputdata.numel());
    for (int i = 0; i < action_num + 2 * lstm_layers * lstm_hidden_size; i++) {
        outputdata_nlp(i) = v[i];
    }

    // outputdata_nlp[14] = 0.0;
    // outputdata_nlp[15] = 0.0;
    // outputdata_nlp[18] = 0.0;
    // outputdata_nlp[19] = 0.0;
    outputdata_nlp_filter.segment(0, 12) = (1.0 - alpha) * outputdata_nlp_filter.segment(0, 12) + alpha * outputdata_nlp.segment(0, 12);
    outputdata_nlp_filter.segment(12, 3) = (1.0 - alpha) * outputdata_nlp_filter.segment(12, 3) + alpha * outputdata_nlp.segment(12, 3);
    // change accordingto action num
    outputdata_nlp_filter.segment(15, 8) = (1.0 - alpha2) * outputdata_nlp_filter.segment(15, 8) + alpha2 * outputdata_nlp.segment(15, 8);
    // std::cout<<"outputdata_nlp_filter: "<<outputdata_nlp_filter.transpose()<<std::endl;

    // std::cout<<"act interface run!"<<std::endl;
}

Eigen::VectorXd NlpModel::get_outputdata() {
    return outputdata_nlp_filter;
}

Eigen::VectorXd NlpModel::get_outputdata_encoder() {
    return outputdata_encoder_nlp;
}

void NlpModel::set_inputdata(Eigen::VectorXd inputdata_nlp_) {

    // set obs
    inputdata_nlp.segment(0, obs_num) = inputdata_nlp_;
    // set hidden states
    if (lstm_layers > 0) {
        inputdata_nlp.segment(obs_num, 2 * lstm_layers * lstm_hidden_size) = outputdata_nlp.segment(action_num, 2 * lstm_layers * lstm_hidden_size);
    }
    // last action for temp
    // inputdata_nlp.segment(52,12) = outputdata_nlp.segment(0,12);
    // set compute flag
    compute_complete = false;
}
