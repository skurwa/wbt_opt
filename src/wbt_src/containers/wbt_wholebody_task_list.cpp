#include <wbt/containers/wbt_wholebody_task_list.hpp>

#include <iostream>
WholeBody_Task_List::WholeBody_Task_List(){}
WholeBody_Task_List::~WholeBody_Task_List(){
	for(size_t i = 0; i < task_list.size(); i++){
		delete task_list[i];
	}
	task_list.clear();
}

void WholeBody_Task_List::append_task(Task* whole_body_task){
	task_list.push_back(whole_body_task);
}

void WholeBody_Task_List::get_task_list_copy(std::vector<Task*>& task_list_out){
	task_list_out = task_list;
}

int WholeBody_Task_List::get_size(){
	return task_list.size();
}

Task* WholeBody_Task_List::get_task(int index){
	if ((index >= 0) && (index < task_list.size())){
		return task_list[index];
	}else{
		std::cerr << "Error retrieving task. Index is out of bounds" << std::endl;
		throw "invalid_index";
	}
}