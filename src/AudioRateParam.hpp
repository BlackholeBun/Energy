class AudioRateParam
{
  public:
    /** Curves are applied to the output signal */
    enum Curve
    {
        LINEAR,      /**< Linear curve */
        EXPONENTIAL, /**< Exponential curve */
        LOGARITHMIC, /**<  Logarithmic curve */
        CUBE,        /**< Cubic curve */
        LAST,        /**< Final enum element. */
    };
    /** Constructor */
    AudioRateParam() {}
    /** Destructor */
    ~AudioRateParam() {}

    /** initialize a parameter using an hid_ctrl object.
    \param input - object containing the direct link to a hardware control source.
    \param min - bottom of range. (when input is 0.0)
        \param max - top of range (when input is 1.0)
    \param curve - the scaling curve for the input->output transformation.
    */
    void Init(int index, float min, float max, Curve curve);

    /** processes the input signal, this should be called at the samplerate of the hid_ctrl passed in.
    \return  a float with the specified transformation applied.
    */
    float Update(float input);

    //Output the current value with the processe method to maintanin compatibility with Daisy Params.
    float Process();

    inline int Index() {return index_;};

    /** 
    \return the current value from the parameter without processing another sample.
    this is useful if you need to use the value multiple times, and don't store

    the output of process in a local variable.
    */
    inline float Value() { return val_; }

  private:
    int           index_;
    float         pmin_, pmax_;
    float         lmin_, lmax_; // for log range
    float         val_;
    Curve         pcurve_;
    float         out_;
};