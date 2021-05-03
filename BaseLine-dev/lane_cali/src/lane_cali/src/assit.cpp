#include "assit.h"
// static const size_t npos = -1;

//modify to auto get str_list value num
vector<string> split_function(string input_str,const char *split_signal){    
    vector<string> result;
    // char *element_p = NULL;
    int pos = 0;
    int count = 0;
    while(input_str.find(split_signal)!=std::string::npos){  
        string splited_ = input_str.substr(pos,input_str.find_first_of(split_signal));  
        // cout<<"splited: "<<splited_.c_str()<<" push_back"<<endl;
        result.push_back(splited_);
        // cout<<"input: "<<input_str.c_str()<<endl;       
        input_str = input_str.substr(input_str.find_first_of(split_signal)+1,input_str.length());
    }
    result.push_back(input_str);
    // cout<<"vec str size: "<< result.size();
    return result;
}




