#include <trivia/trivia_question_handler.h>

/*Read the included text file (CATEGORY_NAME.txt) and store all ques in the variable list_of_ques*/
trivia_question_handler::trivia_question_handler(){
  
}

trivia_question_handler::~trivia_question_handler(){
  
}

void trivia_question_handler::read_file(std::string filePath)
{
  std::ifstream quesStream(filePath.c_str());

  if (quesStream.is_open())
  {
    std::string line;
    std::string ques;
    std::string ans1;
    std::string ans2;
    std::string ans3;
    std::string corr_ans;
    std::string hint;
    
    while ( getline (quesStream,line) )
    {
        if(!line.empty())
        {
              if(line.at(0)=='Q')
              {
                  ques=line.substr(2);
                  boost::trim(ques);
                  getline (quesStream,line);
                  ans1=line.substr(3);
                  boost::trim(ans1);
                  getline (quesStream,line);
                  ans2=line.substr(3);
                  boost::trim(ans2);
                  getline (quesStream,line);
                  ans3=line.substr(3);
                  boost::trim(ans3);
                  getline (quesStream,line);
                  corr_ans=line.substr(3);
                  boost::trim(corr_ans);
                  getline (quesStream, line);
                  hint=line.substr(2);
                  boost::trim(hint);
                  store_ques(ques,ans1,ans2,ans3,corr_ans, hint);
              }
        }
    }
    quesStream.close();
  }
}

/*Store ques in list*/
void trivia_question_handler::store_ques(std::string ques, std::string ans1, std::string ans2, std::string ans3, std::string corr_ans, std::string hint)
{
  if(ques.empty()||ans1.empty()||ans2.empty()||ans3.empty()||corr_ans.empty())
  {
    ROS_ERROR("Ques cannot be stored -- improper formatting of CATEGORY_NAME.txt");
  }else{
    std::vector<std::string> question;
    question.push_back(ques);
    question.push_back(ans1);
    question.push_back(ans2);
    question.push_back(ans3);
    question.push_back(corr_ans);
    question.push_back(hint);
    list_of_ques.push_back(question);
  }
}

/*Retrieve random ques which has not been used in the session already
  Once used, a ques is removed from the stored list_of_ques*/
std::vector<std::string> trivia_question_handler::get_ques()
{
  if(!list_of_ques.empty()){
      int num_ques=list_of_ques.size();
      ROS_INFO("%d ques left", num_ques);
      if(num_ques!=1){
        
        int rand_n=rand() % (num_ques);
        std::vector<std::string> ques=list_of_ques.at(rand_n);
        list_of_ques.erase(list_of_ques.begin()+rand_n);
        
        return ques;
        
      }else if(num_ques==1){
        std::vector<std::string> ques=list_of_ques.at(0);
        list_of_ques.erase(list_of_ques.begin());
        return ques;
        
      }
  }else{
        ROS_WARN("No more ques!");
        std::vector<std::string> empty_ques;
        empty_ques.push_back(" ");
        empty_ques.push_back(" ");
        
        return empty_ques;
  }

}

int trivia_question_handler::get_num_ques(){
  return list_of_ques.size();
}

void trivia_question_handler::delete_progress()
{
  list_of_ques.clear();
}
