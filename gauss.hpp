Mat normalize(Mat matrix, float positiv_sum, float negativ_sum);
float gaussian_function(float x, float y, float sigma);
float gaussian_function_x(float x, float y, float sigma);
float gaussian_function_xx(float x, float y, float sigma);
Mat gaus_filter(float sigma, int kernel_size, float angle);
Mat gaus_filter_xx(float sigma, int kernel_size, float angle);
