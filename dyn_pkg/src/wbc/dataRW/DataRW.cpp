#include "wbc/dataRW/DataRW.h"

namespace wbc_controller_xjm
{
    DataRW::DataRW(){}
    DataRW::~DataRW(){}

    void DataRW::save_vector_to_txt(const Vector_h& vector_need_to_save, string dest_Txt){

        ofstream ofs;
        ofs.open(dest_Txt, std::ios::trunc);
        if(ofs.is_open() == false) {
            std::cout << "保存数据时-打目标文件失败！！！" << std::endl;
        }

        int rows = vector_need_to_save.size();

        for(int i=0; i < rows; i++) {
            ofs << vector_need_to_save(i);            
            ofs <<  endl;
        }
        ofs.close();
    }

    void DataRW::save_matrix_to_txt(const Matrix_h& matrix_need_to_save, string dest_Txt){

        ofstream ofs;
        ofs.open(dest_Txt, std::ios::trunc);
        if(ofs.is_open() == false) {
            std::cout << "保存数据时-打开目标文件失败！！！" << std::endl;
        }

        int rows = matrix_need_to_save.rows();
        int cols = matrix_need_to_save.cols();

        for(int i=0; i < rows; i++) {
            for(int j=0; j < cols; j++)
            {
                ofs << matrix_need_to_save(i,j) << " ";
            }
            ofs <<  endl;
        }
        ofs.close();
    }

    void DataRW::read_txt_as_vector(string sourceTxt, Vector_h& dest_Vector){

        ifstream ifs;
        ifs.open(sourceTxt);
        if(ifs.is_open() == false) {
            std::cout << "读数据时-打开源文件失败！！！" << std::endl;
        }

        int rows = dest_Vector.rows();

        for(int i=0; i < rows; i++) {

            ifs >> dest_Vector(i);
        }
        ifs.close();
    }

    void DataRW::read_txt_as_matrix(string sourceTxt, Matrix_h& dest_Matrix){

        ifstream ifs;
        ifs.open(sourceTxt);
        if(ifs.is_open() == false) {
            std::cout << "读数据时-打开源文件失败！！！" << std::endl;
        }

        int rows = dest_Matrix.rows();
        int cols = dest_Matrix.cols();

        for(int i=0; i < rows; i++) {
            for(int j=0; j < cols; j++)
            {
                ifs >> dest_Matrix(i,j);
            }
        }
        ifs.close();
    }
}
