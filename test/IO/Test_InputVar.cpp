//
// Created by ziqwang on 2020-02-13.
//

#include "IO/InputVar.h"
#include <catch2/catch.hpp>
TEST_CASE("InputVar")
{
    shared_ptr<InputVarList> varList;
    varList = make_shared<InputVarList>();
    REQUIRE(varList->varLists.empty());

    varList->add(0.2, Vector2f(0, 1), "w1", "w1");
    varList->add(0.2, Vector2f(0, 1), "w2", "w2");
    varList->add(0.2, Vector2f(0, 1), "w3", "w3");
    varList->add(0.3, Vector2f(0, 1), "w2", "w2");
    varList->add(0.2, Vector2f(0, 1), "w5", "w5");
    varList->add(0.5, Vector2f(0, 1), "w3", "w3");
    varList->add(0.2, Vector2f(0, 1), "w3", "w3");
    varList->add(0.2, Vector2f(0, 1), "w4", "w4");

    REQUIRE(varList->find("w1") != nullptr);
    REQUIRE(varList->find("w2") != nullptr);
    REQUIRE(varList->find("w3") != nullptr);
    REQUIRE(varList->find("w4") != nullptr);
    REQUIRE(varList->find("w5") != nullptr);
    REQUIRE(varList->find("w6") == nullptr);


    REQUIRE(varList->find("w1")->index == 0);
    REQUIRE(varList->find("w2")->index == 1);
    REQUIRE(varList->find("w3")->index == 2);
    REQUIRE(varList->find("w5")->index == 3);
    REQUIRE(varList->find("w4")->index == 4);
}