// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>

#include "gtest/gtest.h"

#include "rmw_dds_common/security.hpp"

TEST(test_security, files_exist_no_prefix)
{
  std::filesystem::path dir = std::filesystem::path("./test_folder");
  std::filesystem::remove_all(dir);
  EXPECT_TRUE(std::filesystem::create_directories(dir));
  EXPECT_TRUE(std::filesystem::exists(dir));
  EXPECT_TRUE(std::filesystem::is_directory(dir));

  std::array<std::string, 6> required_files = {
    "identity_ca.cert.pem", "cert.pem", "key.pem",
    "permissions_ca.cert.pem", "governance.p7s", "permissions.p7s"
  };
  for (const std::string & filename : required_files) {
    std::filesystem::path full_path = dir / filename;
    std::ofstream output_buffer{full_path.generic_string()};
    output_buffer << "test";
    ASSERT_TRUE(std::filesystem::exists(full_path));
  }

  std::unordered_map<std::string, std::string> security_files;
  ASSERT_TRUE(rmw_dds_common::get_security_files("", dir.generic_string(), security_files));

  EXPECT_EQ(
    security_files["IDENTITY_CA"],
    std::filesystem::path("./test_folder/identity_ca.cert.pem").generic_string());
  EXPECT_EQ(
    security_files["CERTIFICATE"],
    std::filesystem::path("./test_folder/cert.pem").generic_string());
  EXPECT_EQ(
    security_files["PRIVATE_KEY"],
    std::filesystem::path("./test_folder/key.pem").generic_string());
  EXPECT_EQ(
    security_files["PERMISSIONS_CA"],
    std::filesystem::path("./test_folder/permissions_ca.cert.pem").generic_string());
  EXPECT_EQ(
    security_files["GOVERNANCE"],
    std::filesystem::path("./test_folder/governance.p7s").generic_string());
  EXPECT_EQ(
    security_files["PERMISSIONS"],
    std::filesystem::path("./test_folder/permissions.p7s").generic_string());
}

TEST(test_security, files_exist_with_prefix)
{
  std::filesystem::path dir = std::filesystem::path("./test_folder");
  std::filesystem::remove_all(dir);
  EXPECT_TRUE(std::filesystem::create_directories(dir));
  EXPECT_TRUE(std::filesystem::exists(dir));
  EXPECT_TRUE(std::filesystem::is_directory(dir));

  std::array<std::string, 6> required_files = {
    "identity_ca.cert.pem", "cert.pem", "key.pem",
    "permissions_ca.cert.pem", "governance.p7s", "permissions.p7s"
  };
  for (const std::string & filename : required_files) {
    std::filesystem::path full_path = dir / filename;
    std::ofstream output_buffer{full_path.generic_string()};
    output_buffer << "test";
    ASSERT_TRUE(std::filesystem::exists(full_path));
  }

  std::unordered_map<std::string, std::string> security_files;
  ASSERT_TRUE(rmw_dds_common::get_security_files("file://", dir.generic_string(), security_files));

  EXPECT_EQ(
    security_files["IDENTITY_CA"],
    "file://" + std::filesystem::path("./test_folder/identity_ca.cert.pem").generic_string());
  EXPECT_EQ(
    security_files["CERTIFICATE"],
    "file://" + std::filesystem::path("./test_folder/cert.pem").generic_string());
  EXPECT_EQ(
    security_files["PRIVATE_KEY"],
    "file://" + std::filesystem::path("./test_folder/key.pem").generic_string());
  EXPECT_EQ(
    security_files["PERMISSIONS_CA"],
    "file://" + std::filesystem::path("./test_folder/permissions_ca.cert.pem").generic_string());
  EXPECT_EQ(
    security_files["GOVERNANCE"],
    "file://" + std::filesystem::path("./test_folder/governance.p7s").generic_string());
  EXPECT_EQ(
    security_files["PERMISSIONS"],
    "file://" + std::filesystem::path("./test_folder/permissions.p7s").generic_string());
}

TEST(test_security, file_missing)
{
  std::filesystem::path dir = std::filesystem::path("./test_folder");
  std::filesystem::remove_all(dir);
  EXPECT_TRUE(std::filesystem::create_directories(dir));
  EXPECT_TRUE(std::filesystem::exists(dir));
  EXPECT_TRUE(std::filesystem::is_directory(dir));

  std::array<std::string, 5> required_files = {
    "identity_ca.cert.pem", "cert.pem", "key.pem",
    "permissions_ca.cert.pem", "governance.p7s"
  };
  for (const std::string & filename : required_files) {
    std::filesystem::path full_path = dir / filename;
    std::ofstream output_buffer{full_path.generic_string()};
    output_buffer << "test";
    ASSERT_TRUE(std::filesystem::exists(full_path));
  }

  std::unordered_map<std::string, std::string> security_files;
  ASSERT_FALSE(rmw_dds_common::get_security_files("", dir.generic_string(), security_files));
  ASSERT_EQ(security_files.size(), 0UL);
}

TEST(test_security, optional_file_exist)
{
  std::filesystem::path dir = std::filesystem::path("./test_folder");
  std::filesystem::remove_all(dir);
  EXPECT_TRUE(std::filesystem::create_directories(dir));
  EXPECT_TRUE(std::filesystem::exists(dir));
  EXPECT_TRUE(std::filesystem::is_directory(dir));

  std::array<std::string, 7> required_files = {
    "identity_ca.cert.pem", "cert.pem", "key.pem",
    "permissions_ca.cert.pem", "governance.p7s", "permissions.p7s", "crl.pem",
  };
  for (const std::string & filename : required_files) {
    std::filesystem::path full_path = dir / filename;
    std::ofstream output_buffer{full_path.generic_string()};
    output_buffer << "test";
    ASSERT_TRUE(std::filesystem::exists(full_path));
  }

  std::unordered_map<std::string, std::string> security_files;
  ASSERT_TRUE(rmw_dds_common::get_security_files("", dir.generic_string(), security_files));

  EXPECT_EQ(
    security_files["IDENTITY_CA"],
    std::filesystem::path("./test_folder/identity_ca.cert.pem").generic_string());
  EXPECT_EQ(
    security_files["CERTIFICATE"],
    std::filesystem::path("./test_folder/cert.pem").generic_string());
  EXPECT_EQ(
    security_files["PRIVATE_KEY"],
    std::filesystem::path("./test_folder/key.pem").generic_string());
  EXPECT_EQ(
    security_files["PERMISSIONS_CA"],
    std::filesystem::path("./test_folder/permissions_ca.cert.pem").generic_string());
  EXPECT_EQ(
    security_files["GOVERNANCE"],
    std::filesystem::path("./test_folder/governance.p7s").generic_string());
  EXPECT_EQ(
    security_files["PERMISSIONS"],
    std::filesystem::path("./test_folder/permissions.p7s").generic_string());

  EXPECT_EQ(
    security_files["CRL"],
    std::filesystem::path("./test_folder/crl.pem").generic_string());
}
