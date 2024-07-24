#ifndef DATARW_H
#define DATARW_H

#pragma once
#include "para/BikeParameters.hpp"

namespace wbc_controller_xjm
{
    class DataRW
    {
    public:
        DataRW();
        ~DataRW();

        void save_vector_to_txt(const Vector_h& vector_need_to_save, string dest_Txt);
        void save_matrix_to_txt(const Matrix_h& matrix_need_to_save, string dest_Txt);
        void read_txt_as_vector(string sourceTxt, Vector_h& dest_Vector);
        void read_txt_as_matrix(string sourceTxt, Matrix_h& dest_Matrix);

    // private:
    };
}
#endif