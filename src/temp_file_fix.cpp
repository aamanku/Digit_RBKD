
# include <vector>
# include <stdio.h>
# include <stdlib.h>
# include <unistd.h>
# include <cppad/configure.hpp>
# include <cppad/local/temp_file.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS

// #ifndef __has_include
//   static_assert(false, "__has_include not supported");
// #else
// #  if __cplusplus >= 201703L && __has_include(<filesystem>)
// #    include <filesystem>
//      namespace fs = std::filesystem;
// #  elif __has_include(<experimental/filesystem>)
// #    include <experimental/filesystem>
//      namespace fs = std::experimental::filesystem;
// #  elif __has_include(<boost/filesystem.hpp>)
// #    include <boost/filesystem.hpp>
//      namespace fs = boost::filesystem;
// #  endif
// #endif



# define DIR_SEP '/'

namespace CppAD {
namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE

std::string temp_file(void)
{  // path

// using fs = boost::filesystem;
   using boost::filesystem::path;
   //
   // tmp_dir_path
   path tmp_dir_path = boost::filesystem::temp_directory_path();
   //
   // tmp_dir_str
   std::string tmp_dir_str = tmp_dir_path.string();
   if( tmp_dir_str.back() != DIR_SEP )
      tmp_dir_str += DIR_SEP;
   //
   // pattern_str
   std::string pattern_str = tmp_dir_str;
   pattern_str            += "fileXXXXXX";
   //
   // pattern_vec
   std::vector<char> pattern_vec( pattern_str.size() + 1 );
   for(size_t i = 0; i < pattern_str.size(); ++i)
      pattern_vec[i] = pattern_str[i];
   pattern_vec[ pattern_str.size() ] = '\0';
   //
   // fd, pattrern_vec
   int fd = mkstemp( pattern_vec.data() );
   if( fd < 0 )
      return "";
   close(fd);
   //
   // file_name
   std::string file_name = pattern_vec.data();
   return file_name;
}
}
}

