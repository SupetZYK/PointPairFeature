#include <util.h>
#include <string>
#include <string.h>
//double zyk::dot(const float* n1, const float* n2, const int dim)
//{
//	double res = 0;
//	for (int i = 0; i < dim; ++i)
//		res += n1[i] * n2[i];
//	return res;
//}

//double zyk::norm(const float* n, const int dim)
//{
//	double res = 0;
//	for (int i = 0; i < dim; ++i)
//		res += n[i] * n[i];
//	return sqrt(res);
//}

//double zyk::dist(const float* n1, const float* n2, const int dim)
//{
//	double res = 0;
//	for (int i = 0; i < dim; ++i)
//	{
//		double tmp = n1[i]- n2[i];
//		res += tmp*tmp;
//	}
//	return sqrt(res);
//}


#ifndef WIN32
void _splitpath(const char *path, char *drive_, char *dir_, char *fname_, char *ext_)
{
  char *p_whole_name;
  char *drive, *dir, *fname, *ext;
  if(drive_!=NULL)
      drive=drive_;
  else
      drive=new char [10];
  if(dir_!=NULL)
      dir=dir_;
  else
      dir=new char[100];
  if(fname_!=NULL)
      fname=fname_;
  else
      fname=new char[100];
  if(ext_!=NULL)
      ext=ext_;
  else
      ext=new char[50];
  drive[0] = '\0';
  if (NULL == path)
  {
    dir[0] = '\0';
    fname[0] = '\0';
    ext[0] = '\0';
    return;
  }

  if ('/' == path[strlen(path)])
  {
    strcpy(dir, path);
    fname[0] = '\0';
    ext[0] = '\0';
    return;
  }

  p_whole_name = const_cast<char*>(rindex(path, '/'));
  if (NULL != p_whole_name)
  {
    p_whole_name++;
    _split_whole_name(p_whole_name, fname, ext);

    snprintf(dir, p_whole_name - path, "%s", path);
  }
  else
  {
    _split_whole_name(path, fname, ext);
    dir[0] = '\0';
  }
  if(drive_==NULL)
      delete drive;
  if(dir_==NULL)
      delete dir;
  if(fname_==NULL)
      delete fname;
  if(ext_==NULL)
      delete ext;
}

static void _split_whole_name(const char *whole_name, char *fname, char *ext)
{
  char *p_ext;

  p_ext = const_cast<char*>(rindex(whole_name, '.'));
  if (NULL != p_ext)
  {
    strcpy(ext, p_ext);
    snprintf(fname, p_ext - whole_name + 1, "%s", whole_name);
  }
  else
  {
    ext[0] = '\0';
    strcpy(fname, whole_name);
  }
}

#endif
