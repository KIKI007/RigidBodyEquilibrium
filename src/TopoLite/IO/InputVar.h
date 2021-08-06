//
// Created by ziqwang on 21.02.19.
//

#ifndef TOPOLOCKCREATOR_GLUIVAR_H
#define TOPOLOCKCREATOR_GLUIVAR_H
#include <string>
#include <map>
#include <nlohmann/json.hpp>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <iostream>
using std::string;
using std::vector;
using Eigen::Vector2f;
using std::shared_ptr;
using std::make_shared;
using std::map;

enum varValueType{
    VAR_VALUE_NONE,
    VAR_VALUE_BOOL,
    VAR_VALUE_INT,
    VAR_VALUE_FLOAT,
    VAR_VALUE_INTLIST,
    VAR_VALUE_MATRIX4D,
    VAR_VALUE_STRING,
};

enum varGUIType{
    GUI_NONE,
    GUI_SLIDERFLOAT,
    GUI_SLIDERINT,
    GUI_CHECKBOX,
    GUI_TEXTBOX,
};

class InputVar
{
public:
    InputVar()
    {
        var_gui_type = GUI_NONE;
        var_value_type = VAR_VALUE_NONE;
        visible = true;
        update = false;
        func = nullptr;
        index = 0;
    }
    
    ~InputVar(){
        
    }

    InputVar& operator<<(string name){
        var_names.push_back(name);
        return *this;
    }

    void operator =(string name){
        series_name = name;
    }

    virtual void clear_value(){
        return;
    }

    void cloneBase(shared_ptr<InputVar> var){
        var->var_names = var_names;
        var->series_name = series_name;
        var->label = label;
        var->update = update;
        var->var_gui_type = var_gui_type;
        var->var_value_type = var_value_type;
        var->func = func;
        var->index = index;
    }

    virtual shared_ptr<InputVar> clone(){
        shared_ptr<InputVar> var = make_shared<InputVar>();
        cloneBase(var);
        return var;
    }

public:
    vector<string> var_names;
    string series_name;
    string label;
    bool visible;
    bool update;
    int index;
    varGUIType var_gui_type;
    varValueType var_value_type;
    std::function<void()> func;
};

class InputVarInt : public InputVar
{
public:
    InputVarInt(){

    }

    InputVarInt(int val, Vector2f _bound = Vector2f(0, 0))
    {
        default_value = value = val;
        var_gui_type = GUI_SLIDERINT;
        var_value_type = VAR_VALUE_INT;
        bound = _bound;
    }

    ~InputVarInt(){
        clear_value();
    }
    
    void clear_value() override
    {
        value = default_value;
    }

    shared_ptr<InputVar> clone() override{
        shared_ptr<InputVarInt> var = make_shared<InputVarInt>();
        cloneBase(var);
        var->value = value;
        var->default_value = default_value;
        var->bound = bound;
        return var;
    }

    int value;
    int default_value;
    Vector2f bound;
};

class InputVarFloat: public InputVar
{
public:
    InputVarFloat(){

    }

    InputVarFloat(double val, Vector2f _bound = Vector2f(0, 0))
    {
        default_value = value = val;
        var_gui_type = GUI_SLIDERFLOAT;
        var_value_type = VAR_VALUE_FLOAT;
        bound = _bound;
    }
    
    ~InputVarFloat(){
        clear_value();
    }

    void clear_value() override
    {
        value = default_value;
    }

    shared_ptr<InputVar> clone() override{
        shared_ptr<InputVarFloat> var = make_shared<InputVarFloat>();
        cloneBase(var);
        var->value = value;
        var->default_value = default_value;
        var->bound = bound;
        return var;
    }

    double value;
    double default_value;
    Vector2f bound;
};

class InputVarBool: public InputVar{
public:

    InputVarBool(){

    }

    InputVarBool(bool val){
        default_value = value = val;
        var_gui_type = GUI_CHECKBOX;
        var_value_type = VAR_VALUE_BOOL;
    }

    ~InputVarBool(){
        clear_value();
    }
    
    void clear_value() override
    {
        value = default_value;
    }

    shared_ptr<InputVar> clone() override{
        shared_ptr<InputVarBool> var = make_shared<InputVarBool>();
        cloneBase(var);
        var->value = value;
        var->default_value = default_value;
        return var;
    }

    int value;
    int default_value;
};

class InputVarIntList: public InputVar{
public:
    InputVarIntList(){

    }

    InputVarIntList(vector<int> val)
    {
        value = val;
        var_gui_type = GUI_NONE;
        var_value_type = VAR_VALUE_INTLIST;
    }

    shared_ptr<InputVar> clone() override{
        shared_ptr<InputVarIntList> var = make_shared<InputVarIntList>();
        cloneBase(var);
        var->value = value;
        return var;
    }

public:
    vector<int> value;
};

class InputVarMatrix4d: public InputVar{
public:

    InputVarMatrix4d(){

    }

    InputVarMatrix4d(Eigen::Matrix4d val){
        value = val;
        var_gui_type = GUI_NONE;
        var_value_type = VAR_VALUE_MATRIX4D;
    }

    InputVarMatrix4d(vector<double> val){

        vector<double>::iterator it = val.begin();
        for(int id = 0; id < 4; id ++)
        {
            for(int jd = 0; jd < 4; jd++)
            {
                value(id, jd) = *it;
                it++;
            }
        }

        var_gui_type = GUI_NONE;
        var_value_type = VAR_VALUE_MATRIX4D;
    }

    vector<double> to_vector_double(){
        vector<double> data;
        for(int id = 0; id < 4; id ++){
            for(int jd = 0; jd < 4; jd++){
                data.push_back(value(id, jd));
            }
        }
        return data;
    }

    shared_ptr<InputVar> clone() override{
        shared_ptr<InputVarMatrix4d> var = make_shared<InputVarMatrix4d>();
        cloneBase(var);
        var->value = value;
        return var;
    }

public:
    Eigen::Matrix4d value;
};

class InputVarString: public InputVar{
public:
    InputVarString(){

    }

    InputVarString(std::string val)
    {
        value = val;
        var_gui_type = GUI_NONE;
        var_value_type = VAR_VALUE_STRING;
    }

    shared_ptr<InputVar> clone() override{
        shared_ptr<InputVarString> var = make_shared<InputVarString>();
        cloneBase(var);
        var->value = value;
        return var;
    }

public:

    std::string value;
};

class InputVarList
{
public:
    int index;
    std::vector<shared_ptr<InputVar>> varLists;
    map<std::string, InputVar *> varMap;
    string filename;

public:

    InputVarList()
    {
        index = 200;
    }

    InputVarList(const InputVarList &input){
        index = input.index;
        filename = input.filename;

        for(int id = 0; id < input.varLists.size(); id++){
            shared_ptr<InputVar> var = input.varLists[id]->clone();
            varLists.push_back(var);
            varMap[var->var_names.front()] = var.get();
        }
    }

public:
    int getInt(string name)
    {
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarInt *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return int();
    }

    bool getBool(string name)
    {
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarBool *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return bool();
    }

    Vector2f getFloatBound(string name){
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarFloat *) var)->bound;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return Vector2f();
    }

    double getFloat(string name)
    {
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarFloat *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return double();
    }

    Eigen::Matrix4d getMatrix4d(string name)
    {
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarMatrix4d *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return Eigen::Matrix4d();
    }

    vector<int> getIntList(string name)
    {
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarIntList *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return vector<int>();
    }

    std::string getString(string name){
        InputVar* var = find(name);
        if(var != nullptr)
        {
            return ((InputVarString *) var)->value;
        }
        else{
            std::cout << "Error: get a unresgistered variable" << std::endl;
        }
        return std::string();
    }

public:

    vector<shared_ptr<InputVar>> findSeries(string name)
    {
        vector<shared_ptr<InputVar>> series;
        for(shared_ptr<InputVar> var: varLists){
            if(name == var->series_name) series.push_back(var);
        }
        return series;
    }

    InputVar* find(string name) {
        auto find_it = varMap.find(name);
        if (find_it != varMap.end()) return find_it->second;
        return nullptr;
    }

public:

    //int
    InputVar& add(int value, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarInt *)findvar)->value = value;
            return *findvar;
        }
        else{
            shared_ptr<InputVar> var = make_shared<InputVarInt>(value);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    //int
    InputVar& add(int value, Vector2f bound, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarInt *)findvar)->value = value;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarInt>(value, bound);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    //double
    InputVar& add(double value, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarFloat *)findvar)->value = value;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarFloat>(value);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    //double
    InputVar& add(double value, Vector2f bound, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarFloat *)findvar)->value = value;
            ((InputVarFloat *)findvar)->bound = bound;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarFloat>(value, bound);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    //bool
    InputVar& add(bool value, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarBool *)findvar)->value = value;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarBool>(value);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    InputVar& add(vector<int> value, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarIntList *)findvar)->value = value;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarIntList>(value);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    InputVar& add(Eigen::Matrix4d value, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarMatrix4d *)findvar)->value = value;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarMatrix4d>(value);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    InputVar& add(std::string value, string name, string label)
    {
        InputVar *findvar = find(name);
        if(findvar){
            ((InputVarString *)findvar)->value = value;
            return *findvar;
        }
        else {
            shared_ptr<InputVar> var = make_shared<InputVarString>(value);
            *var << name;
            var->label = label;
            var->index = varLists.size();
            varLists.push_back(var);
            varMap[name] = var.get();
            return *var;
        }
    }

    InputVar& add(shared_ptr<InputVar> var)
    {
        if (!var->var_names.empty())
        {
            InputVar *findvar = find(var->var_names.front());
            if(findvar){
                switch(findvar->var_value_type)
                {
                    case VAR_VALUE_INT:
                        ((InputVarInt *)findvar)->value = ((InputVarInt *)(var.get()))->value;
                        ((InputVarInt *)findvar)->bound = ((InputVarInt *)(var.get()))->bound;
                        break;
                    case VAR_VALUE_BOOL:
                        ((InputVarBool *)findvar)->value = ((InputVarBool *)(var.get()))->value;
                        break;
                    case VAR_VALUE_FLOAT:
                        ((InputVarFloat *)findvar)->value = ((InputVarFloat *)(var.get()))->value;
                        ((InputVarFloat *)findvar)->bound = ((InputVarFloat *)(var.get()))->bound;
                        break;
                    case VAR_VALUE_INTLIST:
                        ((InputVarIntList *)findvar)->value = ((InputVarIntList *)(var.get()))->value;
                        break;
                    case VAR_VALUE_MATRIX4D:
                        ((InputVarMatrix4d *)findvar)->value = ((InputVarMatrix4d *)(var.get()))->value;
                        break;
                    case VAR_VALUE_STRING:
                        ((InputVarString *)findvar)->value = ((InputVarString *)(var.get()))->value;
                        break;
                    default:
                        break;
                }
            }
            else {
                var->index = varLists.size();
                varLists.push_back(var);
                varMap[var->var_names.front()] = var.get();
            }
        }
        return *var;
    }

    void clear(){
        varLists.clear();
        varMap.clear();
    }
};

class InputVarManager
{
public:
    std::tuple<nlohmann::json, std::string> getJSON(InputVar *var)
    {
        nlohmann::json node;
        if(var != nullptr){
            node["index"] = var->index;
            node["label"] = var->label;
            node["group"] = var->series_name;
            switch(var->var_value_type)
            {
                case VAR_VALUE_INT:
                    node["value"] = ((InputVarInt *)var)->value;
                    node["type"] = "int";
                    node["range"] = {((InputVarInt *)var)->bound.x(), ((InputVarInt *)var)->bound.y()};
                    break;
                case VAR_VALUE_BOOL:
                    node["value"] = ((InputVarBool *)var)->value;
                    node["type"] = "bool";
                    break;
                case VAR_VALUE_FLOAT:
                    node["value"] = ((InputVarFloat *)var)->value;
                    node["type"] = "double";
                    node["range"] = {((InputVarFloat *)var)->bound.x(), ((InputVarFloat *)var)->bound.y()};
                    break;
                case VAR_VALUE_INTLIST:
                    node["value"] = ((InputVarIntList *)var)->value;
                    node["type"] = "intlist";
                    break;
                case VAR_VALUE_MATRIX4D:
                    node["value"] = ((InputVarMatrix4d *)var)->to_vector_double();
                    node["type"] = "matrix4d";
                    break;
                case VAR_VALUE_STRING:
                    node["value"] = ((InputVarString *)var)->value;
                    node["type"] = "string";
                    break;
                default:
                    break;
            }
        }

        if(var->var_names.empty()){
            return {node, ""};
        }
        else{
            return {node, var->var_names.front()};
        }
    }

    shared_ptr<InputVar> readJSON(const nlohmann::json &json_node, const string name)
    {
        shared_ptr<InputVar> var;

        if(json_node.contains("type"))
        {
            if(json_node["type"] == "int")
            {
                vector<double> bxy = json_node["range"];
                var = make_shared<InputVarInt>((int)json_node["value"],  Vector2f(bxy[0], bxy[1]));
            }
            else if(json_node["type"] == "double"){
                vector<double> bxy = json_node["range"];
                var = make_shared<InputVarFloat>((double)json_node["value"], Vector2f(bxy[0], bxy[1]));
            }
            else if(json_node["type"] == "bool"){
                var = make_shared<InputVarBool>((int)json_node["value"]);
            }
            else if(json_node["type"] == "intlist"){
                var = make_shared<InputVarIntList>(json_node["value"].get<vector<int>>());
            }
            else if(json_node["type"] == "matrix4d"){
                var = make_shared<InputVarMatrix4d>(json_node["value"].get<vector<double>>());
            }
            else if(json_node["type"] == "string"){
                var = make_shared<InputVarString>(json_node["value"].get<std::string>());
            }

            *var << name;
            var->index = json_node["index"];
            var->label = json_node["label"];
            var->series_name = json_node["group"];
        }

        return var;
    }

};

void InitVar_backward(InputVarList *varList);

void InitVar(InputVarList *varList);

void InitVar_Assembly2D(InputVarList *varList);

void InitVar_Assembly3D(InputVarList *varList);

#endif //TOPOLOCKCREATOR_GLUIVAR_H
