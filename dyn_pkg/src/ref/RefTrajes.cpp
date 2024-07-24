#include "ref/RefTrajes.h"

namespace wbc_controller_xjm{

    RefTrajes::RefTrajes(const BikeParameters& conf)
    {
        set_initial_value(conf);
        read_motion_data_fr_txt(conf);
    }
    RefTrajes::~RefTrajes(){}

    /// 初始化
    void RefTrajes::set_initial_value(const BikeParameters& conf){

        double data_N = conf.m_Num;

        m_tra_ref = Vector_h::Zero(conf.m_ntask);
        m_dtra_ref = Vector_h::Zero(conf.m_ntask);
        m_ddtra_ref = Vector_h::Zero(conf.m_ntask);

        m_tra_ref_matrix = Matrix_h::Zero(data_N,conf.m_ntask);
        m_dtra_ref_matrix = Matrix_h::Zero(data_N,conf.m_ntask);
        m_ddtra_ref_matrix = Matrix_h::Zero(data_N,conf.m_ntask);

    }

    void RefTrajes::perform_motion(const BikeParameters& conf, const int & count)
    {
        m_tra_ref = m_tra_ref_matrix.row(count).transpose();
        m_dtra_ref = m_dtra_ref_matrix.row(count).transpose();
        m_ddtra_ref = m_ddtra_ref_matrix.row(count).transpose();

        m_tra_ref(3) = 0.29996175;

        /// ============================================================================================================/// 
        if(count == conf.m_Num-1){
            cout << "==============================================程序运行结束=====================================" << endl;
            cout << "count = " << count << endl;
            cout << "==============================================程序运行结束=====================================" << endl;
        }
        /// ============================================================================================================/// 
    }    

    void RefTrajes::read_motion_data_fr_txt(const BikeParameters& conf){

        string data_name = conf.m_data_name;

        string txt_tra_ref = conf.m_path_ws_src + "/data/"+data_name+"/tra_ref.txt";
        string txt_dtra_ref = conf.m_path_ws_src + "/data/"+data_name+"/dtra_ref.txt";
        string txt_ddtra_ref = conf.m_path_ws_src + "/data/"+data_name+"/ddtra_ref.txt";
    
        m_dataRW.read_txt_as_matrix(txt_tra_ref, m_tra_ref_matrix);
        m_dataRW.read_txt_as_matrix(txt_dtra_ref, m_dtra_ref_matrix);
        m_dataRW.read_txt_as_matrix(txt_ddtra_ref, m_ddtra_ref_matrix);
    
    }
}


