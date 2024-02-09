/*
MIT License

Copyright (c) 2024 Abhijeet Kulkarni

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <casadi/casadi.hpp>
#include <filesystem> // C++17

using namespace casadi;
namespace CasadiEigenCodeGen {
/**
 * @brief The Helper class for code generation for eigen
 */
    class FixedSizeCG : protected CodeGenerator {
    public:
        FixedSizeCG(const std::string &name, const std::string &code_gen_dir, const Dict &opts)
                : CodeGenerator(name, opts), code_gen_dir(code_gen_dir) {
            // Add the include for Eigen
            // to the header
            this->header << "#include <Eigen/Dense>\n";
            // to the source
            this->add_include("Eigen/Dense");
        }

        /** @brief Generates the code using the eigen code generator
         */
        void generate_with_eigen() {
            this->generate();

            // create variables for the generated files
            std::string function_name = this->name;
            bool has_header = this->with_header;
            std::string header_file = function_name + ".h";
            std::string source_file;
            if (this->cpp) {
                source_file = function_name + ".cpp";
            } else {
                source_file = function_name + ".c";
            }
            std::string include_dir = code_gen_dir + "/include";
            std::string src_dir = code_gen_dir + "/src";

            // move the generated files to the code_gen_dir
            // check if the directory exists
            if (!std::filesystem::exists(code_gen_dir)) {
                std::filesystem::create_directory(code_gen_dir);
            }
            if (!std::filesystem::exists(include_dir)) {
                std::filesystem::create_directory(include_dir);
            }
            if (!std::filesystem::exists(src_dir)) {
                std::filesystem::create_directory(src_dir);
            }
            // move the generated files to the code_gen_dir
            std::filesystem::rename(header_file, include_dir + "/" + header_file);
            std::filesystem::rename(source_file, src_dir + "/" + source_file);
        }

        /** @brief Adds a casadi function to the eigen code generator
         * @param f casadi function
         */
        void add_casadi_function(const Function &f) {
            // Add the function to the code generator
            this->add(f);

            // Get argument dimensions and names
            std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> dims = get_casadi_function_io_dims(
                    f);
            std::pair<std::vector<std::string>, std::vector<std::string>> names = get_casadi_function_io_names(f);
            std::vector<std::pair<int, int>> argument_dims = dims.first;
            std::vector<std::pair<int, int>> result_dims = dims.second;
            std::vector<std::string> argument_names = names.first;
            std::vector<std::string> result_names = names.second;
            std::string function_name = f.name();

            // Add the eigen function signature to the header
            this->declare(generate_eigen_function_signature(argument_dims, result_dims, argument_names, result_names,
                                                            function_name));
            // Add the eigen function implementation to the source
            this->body << generate_eigen_function_source(argument_dims, result_dims, argument_names, result_names,
                                                         function_name);
        }


    protected:
        std::string code_gen_dir;

/** @brief Returns the pair of vectors of pairs containing input and output dimensions of the casadi function
 *
 * @param f casadi function
 * @param print if true, prints the input and output dimensions
 * @return {[{rows:cols},{rows:cols}....] : [{rows:cols},{rows:cols}....]}
 */
        std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>>
        get_casadi_function_io_dims(const Function &f, bool print = false) {
            const casadi_int num_args = f.sz_arg();
            const casadi_int num_res = f.sz_res();
            std::vector<std::pair<int, int>> in_dims;
            std::vector<std::pair<int, int>> res_dims;
            for (casadi_int i = 0; i < num_args; ++i) {
                in_dims.push_back({f.sparsity_in(i).rows(), f.sparsity_in(i).columns()});
            }
            for (casadi_int i = 0; i < num_res; ++i) {
                res_dims.push_back({f.sparsity_out(i).rows(), f.sparsity_out(i).columns()});
            }
            if (print) {
                std::cout << "Input dimensions: " << std::endl;
                for (auto &in_dim: in_dims) {
                    std::cout << in_dim.first << "x" << in_dim.second << std::endl;
                }
                std::cout << "Output dimensions: " << std::endl;
                for (auto &res_dim: res_dims) {
                    std::cout << res_dim.first << "x" << res_dim.second << std::endl;
                }
            }
            return {in_dims, res_dims};
        }

        /** @brief Returns pair of vectors containing input and output names of the casadi function
         * @param f casadi function
         * @param print if true, prints the input and output names
         * @return pair of vectors containing input and output names
         */
        std::pair<std::vector<std::string>, std::vector<std::string>>
        get_casadi_function_io_names(const Function &f, bool print = false) {
            const casadi_int num_args = f.sz_arg();
            const casadi_int num_res = f.sz_res();
            std::vector<std::string> in_names;
            std::vector<std::string> res_names;
            for (casadi_int i = 0; i < num_args; ++i) {
                in_names.push_back(f.name_in(i));
            }
            for (casadi_int i = 0; i < num_res; ++i) {
                res_names.push_back(f.name_out(i));
            }
            if (print) {
                std::cout << "Input names: " << std::endl;
                for (auto &in_name: in_names) {
                    std::cout << in_name << std::endl;
                }
                std::cout << "Output names: " << std::endl;
                for (auto &res_name: res_names) {
                    std::cout << res_name << std::endl;
                }
            }
            return {in_names, res_names};
        }

        /** @brief Returns the Eigen function signature
         * @param argument_dims a pair of the form {rows,cols} for each argument
         * @param result_dims a pair of the form {rows,cols} for each result
         * @param argument_names a vector of argument names
         * @param result_names a vector of result names
         * @param function_name name of the function
         * @return Eigen function implementation
         */
        std::string generate_eigen_function_signature(const std::vector<std::pair<int, int>> &argument_dims,
                                                      const std::vector<std::pair<int, int>> &result_dims,
                                                      const std::vector<std::string> &argument_names,
                                                      const std::vector<std::string> &result_names,
                                                      const std::string &function_name) {
            // create a void function signature
            // with arguments as references to Eigen matrices with appropriate dimensions

            std::string function = "void " + function_name + "_eigen(";
            for (int i = 0; i < argument_dims.size(); ++i) {
                if (i != 0) {
                    function += "\t\t\t\t";
                }
                function += "const Eigen::Matrix<casadi_real," + std::to_string(argument_dims[i].first) + "," +
                            std::to_string(argument_dims[i].second) + "> &" + argument_names[i];
                function += ",\n";
            }
            for (int i = 0; i < result_dims.size(); ++i) {
                function += "\t\t\t\tEigen::Matrix<casadi_real," + std::to_string(result_dims[i].first) + "," +
                            std::to_string(result_dims[i].second) + "> &" + result_names[i];
                if (i < result_dims.size() - 1) {
                    function += ", \n";
                }
            }
            function += ")";

            return function;

        }

        /** @brief Returns the Eigen function implementation
         * @param argument_dims a pair of the form {rows,cols} for each argument
         * @param result_dims a pair of the form {rows,cols} for each result
         * @param argument_names a vector of argument names
         * @param result_names a vector of result names
         * @param function_name name of the function
         * @return Eigen function implementation
         */
        std::string generate_eigen_function_source(const std::vector<std::pair<int, int>> &argument_dims,
                                                   const std::vector<std::pair<int, int>> &result_dims,
                                                   const std::vector<std::string> &argument_names,
                                                   const std::vector<std::string> &result_names,
                                                   const std::string &function_name) {
            // create a void function implementation
            std::string source = "extern \"C\" CASADI_SYMBOL_EXPORT ";
            source += generate_eigen_function_signature(argument_dims,
                                                        result_dims,
                                                        argument_names,
                                                        result_names,
                                                        function_name);
            source += " {\n";
            source += "// create arguments for generated casadi function\n";
            source += "const casadi_real *arg[] = {";
            for (int i = 0; i < argument_names.size(); ++i) {
                source += argument_names[i] + ".data()";
                if (i < argument_names.size() - 1) {
                    source += ", ";
                }
            }
            source += "};\n";
            source += "// create results for generated casadi function\n";
            source += "casadi_real *res[] = {";
            for (int i = 0; i < result_names.size(); ++i) {
                source += result_names[i] + ".data()";
                if (i < result_names.size() - 1) {
                    source += ", ";
                }
            }
            source += "};\n";
            source += "// call the generated casadi function\n";
            source += function_name + "(arg, res, nullptr, nullptr, 0);\n";
            source += "}\n";
            return source;
        }
    };


    class DynamicSizeCG : public FixedSizeCG {
    public:
        DynamicSizeCG(const std::string &name, const std::string &code_gen_dir,
                      const Dict &opts)
                : FixedSizeCG(name, code_gen_dir, opts) {}


        /** @brief Adds a casadi function to the eigen code generator
        * @param f casadi function
        */
        void add_casadi_function(const Function &f) {
            // Add the function to the code generator
            this->add(f);

            // Get argument dimensions and names
            std::pair<std::vector<std::pair<int, int>>, std::vector<std::pair<int, int>>> dims = get_casadi_function_io_dims(
                    f);
            std::pair<std::vector<std::string>, std::vector<std::string>> names = get_casadi_function_io_names(f);
            std::vector<std::pair<int, int>> argument_dims = dims.first;
            std::vector<std::pair<int, int>> result_dims = dims.second;
            std::vector<std::string> argument_names = names.first;
            std::vector<std::string> result_names = names.second;
            std::string function_name = f.name();

            // Add the eigen function signature to the header
            this->declare(generate_eigen_function_signature(argument_names, result_names, function_name));
            // Add the eigen function implementation to the source
            this->body << generate_eigen_function_source(argument_names, result_names, function_name);
        }

    private:

        /** @brief Returns the Eigen function signature
         * @param argument_names a vector of argument names
         * @param result_names a vector of result names
         * @param function_name name of the function
         * @return Eigen function implementation
         */
        std::string generate_eigen_function_signature(const std::vector<std::string> &argument_names,
                                                      const std::vector<std::string> &result_names,
                                                      const std::string &function_name) {
            // create a void function signature
            std::string function = "void " + function_name + "_eigen (";
            for (int i = 0; i < argument_names.size(); ++i) {
                if (i != 0) {
                    function += "\t\t\t\t";
                }
                function += "const Eigen::Matrix<casadi_real,-1,-1> &" + argument_names[i];
                function += ",\n";
            }
            for (int i = 0; i < result_names.size(); ++i) {
                function += "\t\t\t\tEigen::Matrix<casadi_real,-1,-1> &" + result_names[i];
                if (i < result_names.size() - 1) {
                    function += ", \n";
                }
            }
            function += ")";

            return function;

        }


        /** @brief Returns the Eigen function implementation
         * @param argument_names a vector of argument names
         * @param result_names a vector of result names
         * @param function_name name of the function
         * @return Eigen function implementation
         */
        std::string generate_eigen_function_source(const std::vector<std::string> &argument_names,
                                                   const std::vector<std::string> &result_names,
                                                   const std::string &function_name) {
            // create a void function implementation
            std::string source = "extern \"C\" CASADI_SYMBOL_EXPORT ";
            source += generate_eigen_function_signature(argument_names,
                                                        result_names,
                                                        function_name);
            source += " {\n";
            source += "// create arguments for generated casadi function\n";
            source += "const casadi_real *arg[] = {";
            for (int i = 0; i < argument_names.size(); ++i) {
                source += argument_names[i] + ".data()";
                if (i < argument_names.size() - 1) {
                    source += ", ";
                }
            }
            source += "};\n";
            source += "// create results for generated casadi function\n";
            source += "casadi_real *res[] = {";
            for (int i = 0; i < result_names.size(); ++i) {
                source += result_names[i] + ".data()";
                if (i < result_names.size() - 1) {
                    source += ", ";
                }
            }
            source += "};\n";
            source += "// call the generated casadi function\n";
            source += function_name + "(arg, res, nullptr, nullptr, 0);\n";
            source += "}\n";
            return source;
        }


    };
};