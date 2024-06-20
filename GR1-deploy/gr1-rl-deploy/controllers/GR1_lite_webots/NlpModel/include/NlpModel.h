#ifndef NLPMODEL_H_
#define NLPMODEL_H_
/**
 * @file nlpmodel.h
 * @author ren xiaoyu (xiaoyu.ren@fftai.com)
 * @brief
 * net work infrence
 * @version 0.1
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <Eigen/Dense>
#include <thread>
#include <time.h>
#include <torch/script.h>
// for lstm
class NlpModel {
  public:
    NlpModel();
    ~NlpModel();
    void init(std::string model_pb_path, std::string encoder_pb_path, int obs_num_, int encoder_num_, int action_num_, int latent_num_, int lstm_layers_, int lstm_hidden_size_);
    void act_interface();
    Eigen::VectorXd act_interface_encoder(Eigen::VectorXd inputdata_encoder_nlp);
    void set_inputdata(Eigen::VectorXd inputdata_nlp_);
    // void set_inputdata_encoder(Eigen::VectorXd inputdata_nlp_);
    Eigen::VectorXd get_outputdata();
    Eigen::VectorXd get_outputdata_encoder();

  private:
    //
    Eigen::VectorXd inputdata_nlp;
    Eigen::VectorXd inputdata_encoder_nlp;
    Eigen::VectorXd outputdata_nlp;
    Eigen::VectorXd outputdata_encoder_nlp;
    Eigen::VectorXd outputdata_nlp_filter;
    int obs_num;
    int encoder_num;
    int action_num;
    int latent_num;
    int lstm_layers;
    int lstm_hidden_size;
    // std::string model_pb;
    bool compute_complete = true;
    std::thread nlpmodel_thread;
    //
    torch::jit::script::Module nlpmodel_;
    torch::jit::script::Module encodermodel_;
    //
    double alpha = 1;
    double alpha2 = 1;
};
#endif