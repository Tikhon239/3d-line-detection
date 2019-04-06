pair<Point2f,Point2f> first_end(seg &segment);
pair <float, float> line_model_fit(seg &segment);
void line_model_fit_and_image(seg &segment, Mat &image);
void segment_image(l &line, vector< pair<float, float> > &line_coefs, Mat &image);
bool merge(l &line1, vector< pair<float,float> >&coefs1, l &line2, vector <pair<float,float> > &coefs2);
