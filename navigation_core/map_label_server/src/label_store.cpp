// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <regex>

#include "map_label_server/label_store.hpp"

namespace cyberdog
{
namespace navigation
{

const std::string kMapLabelDirectory = "/home/yzyrobot/workspace/robot/demo/mapping/";

LabelStore::LabelStore()
 : map_label_directory_{kMapLabelDirectory}
{
  // LoadLabels(map_label_directory_);

  Debug();
}

LabelStore::~LabelStore()
{
}

void LabelStore::AddLabel(
  const std::string & filename,
  const std::string & label_name,
  const protocol::msg::Label::SharedPtr label)
{
  auto doc = ToJson(label);
  rapidjson::Document label_json(rapidjson::kObjectType);
  common::CyberdogJson::Add(label_json, label_name, doc);
  common::CyberdogJson::WriteJsonToFile(filename, label_json);
}

void LabelStore::AddLabel(
  const std::string & filename,
  const std::string & label_name,
  const protocol::msg::Label::SharedPtr label,
  rapidjson::Document & doc)
{
  auto tmp_doc = ToJson(label);
  common::CyberdogJson::Add(doc, label_name, tmp_doc);
}

//
  //delete a Label in the Labels
void LabelStore::DeleteLabel(
  const std::string & filename,
  const std::string & label_name,
  rapidjson::Document & existed_doc
)
{
  //check current "*.json" is existed or not
 std::string label_filename = map_label_directory() + filename;

  if (IsExist(label_filename)) 
  {
    return ;
  }

  for (auto it = existed_doc.MemberBegin(); it != existed_doc.MemberEnd(); ++it) 
  {
    if (it->name.GetString() == label_name) 
    {
       // delete labelName,physicX and physicY
        // label_name is key， but func " RemoveMember(ch*) " just accept " ch* ", so "std::string" must be convertd to " char* " by func " c_str() "  
        existed_doc.RemoveMember( (char*)label_name.c_str() ); 
        break;
    }
  }

  //or
  // rapidjson::Value::ConstValueIterator it = existed_doc.FindMember( (char*)label_name.c_str() );

  // if(it != existed_doc.MemberEnd())
  // {
  //   existed_doc.RemoveMember( (char*)label_name.c_str() ); 
  // }

}

void LabelStore::ChangeLable(
  const std::string & old_label_name,
  const std::string & new_label_name,
  const protocol::msg::Label::SharedPtr & new_label,
  rapidjson::Document & existed_doc
  )
{
   std::string label_filename = map_label_directory() + "test.json";

  if ( IsExist(label_filename) ) // 若label_filename存在 则返回false，不执行return
  {
    return ;
  }

  //change a label
    existed_doc.RemoveMember( (char*)old_label_name.c_str() );
    
    AddLabel(label_filename, new_label_name, new_label, existed_doc);
}

// cheak a label is existed or not in the labels
bool LabelStore::IsLabelExist(
  const std::string & filename,
  const std::string & label_name,
  rapidjson::Document & existed_doc
)
{
  //check current "*.json" is existed or not
 std::string label_filename = map_label_directory() + filename;
 
  if ( IsExist(label_filename) ) // if label_filename exist, the func "IsExist(label_filename)" would return false.
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the .json is not existed" );
    return false;
  }

  for (auto it = existed_doc.MemberBegin(); it != existed_doc.MemberEnd(); ++it) {
    if (it->name.GetString() == label_name) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the label is existed in the .json" );
      return true;
    }

    if (it == (existed_doc.MemberEnd()-1)) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "the label is not existed in the .json" );
      return false;
    }
  }
}

//para “directory” don't use temporarily
bool LabelStore::CreateMapLabelFile(
  const std::string & directory, 
  const std::string & filename)
{
  std::string label_filename = map_label_directory() + filename;
  if (IsExist(label_filename)) {
    return false;
  }

  std::fstream ofs;
  ofs.open(label_filename.c_str(), std::ios::app);

  if (!ofs) {
      std::cerr << "Could not open " << filename << "." << std::endl;
      ERROR("Could not open %s.", label_filename);
      return false;
  }
  ofs.close();
  return true;
}

bool LabelStore::DeleteMapLabelFile(const std::string & filename)
{
  return filesystem::remove(
    filesystem::path(map_label_directory() + filename));
}

bool LabelStore::IsExist(const std::string & filename)
{
  return filesystem::exists(
    filesystem::path(map_label_directory() + filename));
}

std::string LabelStore::GetLabelsFilenameFromMap(const std::string & map_name)
{
  std::string label_filename = "";
  auto it = labels_name_.find(map_name);
  if (it != labels_name_.end()) {
    label_filename = it->second;
  }
  return label_filename;
}

std::string LabelStore::map_label_directory() const
{
  return map_label_directory_;
}

void LabelStore::SetMapName(
  const std::string & label_filename,
  const std::string & map_filename,
  rapidjson::Document & doc)
{
  common::CyberdogJson::Add(doc, "map_name", map_filename);
}

bool LabelStore::LoadLabels(const std::string & directory)
{
  if (!filesystem::exists(directory)) {
    ERROR("directory is not exist.");
    return false;
  }

  std::regex file_suffix("(.*)(.json)");// *.json
  filesystem::path path(directory);
  for (auto filename : filesystem::directory_iterator(path)) {
    if (std::regex_match(filename.path().c_str(), file_suffix)) {
      INFO("filename : %s", filename.path().c_str());
    }
  }
  return true;
}

void LabelStore::Write(const std::string & label_filename, const rapidjson::Document & doc)
{
  common::CyberdogJson::WriteJsonToFile(label_filename, doc);
}

void LabelStore::Read(
  const std::string & label_filename,
  std::vector<protocol::msg::Label> & labels)
{
  if (!filesystem::exists(label_filename)) {
    ERROR("label_filename is not exist.");
    return;
  }

  rapidjson::Document document(rapidjson::kObjectType);
  common::CyberdogJson::ReadJsonFromFile(label_filename, document);

  for (auto it = document.MemberBegin(); it != document.MemberEnd(); ++it) 
  {
    if (it->name.GetString() == "map_name" || it->value.IsString()) {
      continue;
    }

    INFO("----------------------------------------");
    INFO("key = %s, x = %f, y = %f", it->name.GetString(), 
      it->value["x"].GetFloat(), it->value["y"].GetFloat());

    auto label = std::make_shared<protocol::msg::Label>();

    //label->set__label_name(it->name.GetString());
    label->set__physic_x(it->value["x"].GetFloat());
    label->set__physic_y(it->value["y"].GetFloat());
    labels.emplace_back(*label.get());
  }
}

bool LabelStore::ReadLabels(const std::string & label_filename, rapidjson::Document & doc)
{
  if (!filesystem::exists(label_filename)) {
    ERROR("label_filename is not exist.");
    return false;
  }

  // rapidjson::Document document(rapidjson::kObjectType);
  return common::CyberdogJson::ReadJsonFromFile(label_filename, doc);
}

void LabelStore::Debug()
{
  // CreateMapLabelFile(map_label_directory(), "test.json");
  std::string label_filename = map_label_directory() + "test.json";
  rapidjson::Document doc(rapidjson::kObjectType);

  // mapname
  // SetMapName(label_filename, "test.pgm", doc);

  // sofa
  auto sofa = std::make_shared<protocol::msg::Label>();
  sofa->set__physic_x(1.23);
  sofa->set__physic_y(2.25);
  AddLabel(label_filename, "sofa", sofa, doc);

  // auto sofa2 = std::make_shared<protocol::msg::Label>();
  // sofa2->set__physic_x(1.29);
  // sofa2->set__physic_y(1.23);
  // AddLabel(label_filename, "sofa2", sofa2, doc);

  // // bed
  // auto bed = std::make_shared<protocol::msg::Label>();
  // bed->set__physic_x(3.23);
  // bed->set__physic_y(3.25);
  // AddLabel(label_filename, "bed", bed, doc);

  // // bed
  // auto bed2 = std::make_shared<protocol::msg::Label>();
  // bed2->set__physic_x(5.23);
  // bed2->set__physic_y(5.25);
  // AddLabel(label_filename, "bed2", bed2, doc);

  // // bed
  // auto bed3 = std::make_shared<protocol::msg::Label>();
  // bed3->set__physic_x(3.23);
  // bed3->set__physic_y(3.25);
  // AddLabel(label_filename, "bed3", bed3, doc);

  //cheack a label is existed or not
  //  const std::string checkingLabel = "bed4";
  // IsLabelExist(label_filename, checkingLabel, doc);
  ReadLabels(label_filename, doc);
  const std::string checkingLabel = "newBed";
  IsLabelExist(label_filename, checkingLabel, doc);
 
  //delete a label
  const std::string deletingLabel = "newBed";
  DeleteLabel(label_filename, deletingLabel, doc);

  //change a label named "bed" 
    std::string OldLabelName = "newBed2";
    std::string NewLabelName = "bed";

    auto newBedPtr = std::make_shared<protocol::msg::Label>(); 
    newBedPtr->set__physic_x(15.99);
    newBedPtr->set__physic_y(15.99);
    
    ChangeLable(OldLabelName, NewLabelName, newBedPtr, doc);

  // save
  Write(label_filename, doc);

  // test read
  std::vector<protocol::msg::Label> labels;
  Read(label_filename, labels);
}

rapidjson::Document LabelStore::ToJson(const protocol::msg::Label::SharedPtr label)
{
  rapidjson::Document label_json(rapidjson::kObjectType);
  common::CyberdogJson::Add(label_json, "x", label->physic_x);
  common::CyberdogJson::Add(label_json, "y", label->physic_y);
  return label_json;
}

}   //  namespace navigation
}   //  namespace cyberdog
