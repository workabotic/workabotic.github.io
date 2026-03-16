---
title: "Embedded Digit Recognition Pipeline"
date: 2026-03-01 10:00:00
description: Training, quantizing, and embedding a Convolutional Neural Network in a STM32 microcontroller for real-time digit recognition.
author: lucasmazzetto
keywords: computer vision, neural network, convolutional neural network, deep learning, quantization, microcontroller, embedded, arm, STM32, stm32, embedded systems, electrical engineering.
---

## Introduction 

Microcontrollers are everywhere, from IoT sensors and small actuators to mobile robots that must run on a battery and react in real time. However, they usually have tight limits in RAM and flash, clock speed, and sometimes they don't even have a floating-point unit (FPU) for fast float operations.

On the other side, neural networks are very computationally expensive. They need many multiplications and a lot of memory bandwidth, and they usually store weights as `float32`. On a microcontroller this is a problem because `float32` takes space (4 bytes) and, without an FPU, float operations can be slow and waste energy, which is exactly what you do not want on real-time and battery-powered systems.

Even though microcontrollers and neural networks may seem far apart, we can bring them closer with techniques like quantization. With `int8` quantization, we reduce the model size and speed up inference by replacing float weights with 8-bit integers, while keeping scale factors to preserve the original numeric range. In practice, this can shrink the footprint by four times and even make the model fast enough to run on a microcontroller.

This article presents a simple proof of concept. A Convolutional Neural Network (CNN) was trained for digit recognition, quantized from `float32` to `int8`, exported with quantized weights and scale constants, and executed on an STM32 microcontroller using a fixed-point inference pipeline.

The entire source code for this project is available on [GitHub](https://github.com/workabotic/stm32_digit_recognition). You can explore it, reuse it in your own projects, and contribute if you want.

### Quantization

Quantization maps numerical values from a large set to a smaller set. In neural networks, it is a practical way to make the models smaller and faster by replacing `float32` values with lower-precision values. There are two common ways to do this: post-training quantization (PTQ) and quantization-aware training (QAT). PTQ is simpler and faster to apply, but it can lose more accuracy, while QAT usually preserves accuracy better but requires retraining the neural network and a more complex workflow. In PTQ, the model is trained in `float32`, and quantization is applied only after training. A short calibration step is usually used to measure activation ranges and choose the right quantization scale factors.

For embedded inference, the most common target is integer quantization, usually `int8`. This is attractive because a `float32` value takes 4 bytes, while an `int8` value takes only 1 byte, which reduces the memory footprint and bandwidth by about four times. It also allows inference to run entirely with integer operations, which are typically faster and more efficient than floating-point operations on small MCUs.

After choosing when quantization happens (PTQ or QAT), it is also necessary to choose how values are mapped into integers. Two common approaches are symmetric and affine (asymmetric) quantization. In symmetric quantization, the integer range is centered at zero, so zero maps to zero and the conversion uses only a scale factor. In affine quantization, a zero-point is added to shift the integer range, which is useful when values are not centered around zero (for example, many activations). This choice affects the quantization error and helps keep the `int8` model closer to the original `float32` behavior.

#### Asymmetric Quantization

The basic idea is to map a real value $$x$$ into an integer $$\hat{x}$$ using a scale and, optionally, a zero-point offset. For asymmetric quantization, the mapping is:

$$
\hat{x} = \operatorname{clip}\left(\operatorname{round}(x \cdot s) + z,\; q_{\min},\; q_{\max}\right)
$$

$$
x \approx (\hat{x} - z) \cdot s^{-1}
$$

where $$s > 0$$ is the quantization multiplier and $$z$$ is the zero-point. In this formulation, quantization is performed by multiplying the real value by $$s$$ and rounding to the nearest integer, while reconstruction uses the inverse scale $$s^{-1}$$. Also, $$\operatorname{round}(v)$$ denotes rounding $$v$$ to the nearest integer, and $$\operatorname{clip}(v, q_{\min}, q_{\max})$$ clamps $$v$$ to the allowed integer range (values below $$q_{\min}$$ become $$q_{\min}$$, and values above $$q_{\max}$$ become $$q_{\max}$$).

If the real range is $$[r_{\min}, r_{\max}]$$ and the integer range is $$[q_{\min}, q_{\max}]$$, we typically choose:

$$
\begin{aligned}
s &= \frac{q_{\max} - q_{\min}}{r_{\max} - r_{\min}}
\\
\\
z &= \operatorname{round}\left(q_{\min} - r_{\min} \cdot s\right)
\end{aligned}
$$

The scale $$s$$ acts as a quantization multiplier, converting real values into the integer domain, while the zero-point $$z$$ shifts the integer grid so that zero in the real domain can be represented as accurately as possible. This is why affine quantization is useful for activations that are not centered at zero: it can shift the integer grid to reduce error.

![Asymmetric Quantization Schema]({{ site.url }}{{ site.baseurl }}/public/images/embedded-digit-recognition-pipeline/asymmetric_quantization_schema.webp)
*Figure 1 — Asymmetric Quantization Schema.*

#### Symmetric Quantization


Now we can specialize the affine mapping to symmetric quantization by constraining the zero-point to zero, that is, ($$z = 0$$):

$$
\hat{x} = \operatorname{clip}\left(\operatorname{round}(x \cdot s),\; -Q,\; Q\right)
$$

$$
x \approx \hat{x} \cdot s^{-1}
$$

In this formulation the quantized integer values are restricted to the symmetric range $$[-Q, Q]$$, where $$Q$$ represents the maximum magnitude representable by the chosen integer format. To determine the value of $$Q$$ we need to consider how signed integers are represented. In the two's-complement convention, a signed integer type with $$b$$ bits has the representable range:

$$
[-2^{b-1},\; 2^{b-1}-1]
$$

However, symmetric quantization typically uses the slightly smaller interval so that the positive and negative ranges have the same magnitude:

$$
[-Q, Q], \qquad Q = (2^{b-1}-1)
$$

The most negative value $$-2^{b-1}$$ is avoided because it has no positive counterpart and would break the symmetry around zero. For example, with signed `int8` ($$b=8$$) the representable range is $$[-128, 127]$$. But, to maintain symmetry we choose:

$$
Q = 2^{(8-1)}-1 = 127
$$

Which yields the symmetric interval $$[-127, 127]$$.

Once the integer range is defined, symmetric quantization must determine the magnitude parameter $$\alpha$$ that represents the range of real values to be mapped into this interval. Given a real interval $$[r_{\min}, r_{\max}]$$, the parameter $$\alpha$$ defines the symmetric range $$[-\alpha,\alpha]$$ that will be represented in the quantized domain.

In practice, the value of $$\alpha$$ is estimated during a calibration step, where representative samples from the dataset are passed through the model and the resulting activations are observed. This calibration process approximates the typical activation range produced during inference, allowing the integer grid $$[-Q,Q]$$ to be aligned with the values that actually occur in the model. Choosing $$\alpha$$ based on these observed statistics helps reduce quantization error and improves the accuracy of the quantized network.

Once $$\alpha$$ has been determined, the real interval $$[-\alpha,\alpha]$$ is mapped onto the integer interval $$[-Q,Q]$$. The quantization factor is therefore:

$$
s = \frac{Q}{\alpha}
$$

For instance, for signed `int8`, where $$Q=127$$, this becomes:

$$
s = \frac{127}{\alpha}
$$

Symmetric quantization can therefore be interpreted as a constrained affine quantization with zero offset. Removing the zero-point eliminates the shift term in the arithmetic expressions, which can simplify the integer computations performed during inference.

![Symmetric Quantization Schema]({{ site.url }}{{ site.baseurl }}/public/images/embedded-digit-recognition-pipeline/symmetric_quantization_schema.webp)
*Figure 2 — Symmetric Quantization Schema.*

#### Quantization Granularity

Another important aspect of quantized neural networks is the granularity used when assigning scale factors. Granularity defines how many elements of a tensor share the same quantization parameters. 

The choice of granularity involves a trade-off between numerical accuracy and computational simplicity. Sharing the same quantization parameters across many elements simplifies the implementation and reduces computational cost, since fewer scale values must be stored and applied. However, this may increase quantization error because a single scale must represent values with different ranges. Using finer granularity allows scale parameters to better match the local distribution of values, reducing quantization error, but at the cost of additional parameters and more rescaling operations during inference.

In per-tensor quantization, a single scale parameter is used for all elements of a tensor. If $$x$$ is the input activation tensor, the quantized representation is therefore computed element-wise as:

$$
\hat{x} = \operatorname{round}(x \cdot s)
$$

where $$s$$ is shared by every element of the tensor. This approach is simple and efficient because the same scale factor can be applied to all values.

However, tensors such as weight matrices often exhibit different numerical ranges across their rows or channels. To better preserve these variations, a finer granularity can be used. In per-output-feature quantization, each output column of the weight matrix has its own scale factor:

$$
\hat{w}_{k,m} = \operatorname{round}(w_{k,m} \cdot s_{m})
$$

where $$s_m$$ is the scale factor associated with output feature $$m$$, shared by all elements of column $$m$$ of the weight matrix.

In convolutional layers, a similar strategy is used but the granularity is typically defined per output channel. Each convolution filter corresponds to one output channel, and different filters may have very different numerical ranges. Using a single scale for all filters could therefore introduce large quantization errors. In per-channel quantization, each output channel is assigned its own scale factor, allowing the quantization to better match the distribution of values within each filter while still keeping the computation efficient.

If $$w_{c,k,i,j}$$ denotes the weight of a convolution kernel for output channel $$c$$, input channel $$k$$, and spatial coordinates $$(i,j)$$, the quantized representation is computed element-wise as:

$$
\hat{w}_{c,k,i,j} = \operatorname{round}(w_{c,k,i,j} \cdot s_c)
$$

where $$s_c$$ is the scale factor associated with output channel $$c$$, shared by all elements of the filter corresponding to that channel.


### Calibration

In PTQ, the scale parameters must be chosen so that the integer grid covers the range of values that actually appear during inference. During this process, a representative subset of the dataset is passed through the trained floating-point model and statistics of the activations are collected. These statistics are then used to estimate the clipping bound that defines the quantization scale.

Several strategies can be used to estimate this bound. The simplest is maximum value calibration, which sets the bound equal to the largest observed activation magnitude during calibration. A more robust alternative is percentile calibration, where the clipping threshold is chosen as a high percentile of the observed magnitude distribution, intentionally discarding a small fraction of extreme values. Histogram-based calibration builds a histogram of activation magnitudes to approximate their probability distribution. KL-divergence (entropy) calibration is another method that selects the clipping threshold by minimizing the Kullback–Leibler divergence between the original activation distribution and a simulated quantized distribution.

![Calibration Methods]({{ site.url }}{{ site.baseurl }}/public/images/embedded-digit-recognition-pipeline/calibration_methods.webp)
*Figure 3 — Calibration Methods (image extracted from [Hao et al. 2020](https://arxiv.org/abs/2004.09602)).*

#### Maximum value calibration

The simplest approach is maximum calibration, which sets the clipping bound equal to the largest observed absolute value:

$$
\alpha = \max |x|
$$

This method guarantees that all observed values are representable without clipping. Although simple and safe, max calibration can be sensitive to outliers, since a single large value may increase the scale and reduce the effective precision for the majority of values.

#### Percentile calibration

Percentile calibration determines the clipping bound by selecting a high percentile of the observed activation magnitude distribution. Instead of using the maximum value, this method defines the clipping range so that a small fraction of extreme values is ignored. If $$p$$ denotes the chosen percentile, the clipping bound is defined as:

$$
\alpha = \text{percentile}(|x|, p)
$$

By discarding rare outlier values, percentile calibration allocates more of the available integer levels to the range where most activations occur. This often improves the effective quantization resolution compared to maximum calibration, especially when the activation distribution contains a few large but infrequent values.

#### Histogram-based calibration

Histogram-based calibration estimates the dynamic range of activations by approximating their statistical distribution. During calibration, a representative dataset is passed through the model and the values observed at the inputs of each quantized layer are collected. These values are used to build a histogram that approximates the probability distribution of the activations, denoted by $$p(x)$$.

Using this estimated distribution, the calibration step determines a suitable range parameter $$\alpha$$ that represents the magnitude of the activations to be preserved during quantization. Unlike maximum calibration, which uses only the largest observed value, histogram-based calibration uses the full distribution $$p(x)$$ to estimate a range that better reflects how the activations are typically distributed across the dataset.

#### KL-divergence calibration

An even more refined method selects the clipping threshold by minimizing the Kullback–Leibler (KL) divergence between the original activation distribution and a simulated quantized distribution. Let $$p(x)$$ be the normalized histogram of the activation magnitudes. For each candidate threshold $$T$$, values above the threshold are folded into the last histogram bin, producing a truncated distribution. This distribution is then quantized into $$N$$ bins (typically $$N=128$$ for `int8`) and reconstructed back to the original resolution, producing an approximate distribution $$q(x)$$.

The optimal threshold is chosen as:

$$
\alpha = \arg\min_T D_{\mathrm{KL}}(p \parallel q)
$$

Where:

$$
D_{\mathrm{KL}}(p \parallel q) = \sum_i p_i \log\left(\frac{p_i}{q_i}\right)
$$

This method often performs better than maximum calibration because it allows a small amount of distortion in the tails of the distribution in order to better preserve the regions where most activation values occur.

### Integer-only Inference

After quantization and calibration, the model parameters are associated with scale factors and clipping ranges that define how real value activations and weights are represented in the integer domain. 

However, to fully benefit from quantization on embedded hardware, the forward pass of the neural network must be executed using integer arithmetic only. This requires adapting the neural network operations so they can operate directly on quantized integers. In convolutional neural networks, this includes reformulating convolution operations, matrix multiplications from linear layers, pooling operations, and activation functions to work in the integer domain. In addition, the floating-point scaling parameters obtained during calibration must be converted to fixed-point representations so that the rescaling steps required during inference can also be performed using integer arithmetic.

In practice, each layer of the network follows a similar computational structure during integer-only inference. The input activations, represented in Q16 fixed-point format, are first quantized to the `int8` domain using the appropriate scale factor. The core neural network operation is then performed using integer arithmetic, producing accumulators typically stored in a wider integer format. Finally, the result is rescaled back to the Q16 fixed-point domain using the inverse scale factors so that the output can be used as the input to the next layer.

#### Fixed-Point Arithmetic

In a fixed-point representation, a real value $$x$$ is first converted into an integer by multiplying it by a constant scaling factor that corresponds to a fixed number of fractional bits. Using the Q-format notation, a value stored in $$Q_n$$ format means that the integer encodes a real number with $$n$$ fractional bits. The integer representation is therefore obtained as:

$$
x^{(Q_n)} = \operatorname{round}\left(x \cdot 2^{n}\right)
$$

Once the value is stored in fixed-point form, the original real value can be recovered by applying the inverse scaling operation. In this case, the integer is divided by the same power-of-two factor used during encoding:

$$
x \approx \frac{x^{(Q_n)}}{2^{n}}
$$


Because the scaling factor is a power of two, these conversions can be implemented efficiently using integer multiplications and bit shifts. This representation allows real-valued computations to be approximated using integer arithmetic. Basic operations follow simple scaling rules. Addition and subtraction are straightforward when operands share the same format:

$$
x^{(Q_n)} + y^{(Q_n)} = (x+y)^{(Q_n)}
$$

Multiplication produces a result with doubled fractional precision:

$$
x^{(Q_n)} \cdot y^{(Q_n)} = (xy)^{(Q_{2n})}
$$

To restore the original format, the result must be shifted right by $$n$$ bits:

$$
(xy)^{(Q_n)} \approx
\operatorname{round}\!\left(
\frac{x^{(Q_n)} \cdot y^{(Q_n)}}{2^{n}}
\right)
$$

Using fixed-point representation allows real values to be encoded as integers while preserving fractional precision. In integer-only neural network inference, not only activations and intermediate results are stored in fixed-point format, but also the scale factors and other parameters obtained during quantization. By representing these values as fixed-point integers, all operations in the forward pass, such as multiplications, accumulations, and rescaling, can be implemented using integer arithmetic only, eliminating the need for floating-point computation while still approximating the original behavior of the network.

#### Matrix Multiplication for Linear Layers

In a linear layer, the general feedforward operation includes both a matrix
multiplication and a bias term:

$$
y_{n,m} = \sum_{k=0}^{K-1} x_{n,k}\, w_{k,m} + b_m
$$

Where $$x_{n,k}$$ denotes the element of the input matrix at row $$n$$ and column $$k$$, and $$w_{k,m}$$ denotes the element of the weight matrix at row $$k$$ and column $$m$$. The index $$k$$ represents the shared dimension of the matrix multiplication, and the summation accumulates the products between the corresponding elements of the input and weight matrices. The term $$b_m$$ is the bias associated with output column $$m$$, and $$K$$ denotes the size of the shared dimension between the two matrices. The indices $$n$$ and $$m$$ respectively denote the row and column of the resulting output matrix.

However, the bias term can be removed during training to simplify the arithmetic used during inference. In deep neural networks with multiple layers and nonlinear activations, biases are not strictly required because shifts in the activation distribution can be learned through combinations of weights and nonlinearities. Removing the bias therefore reduces the number of parameters and simplifies the computation, while also avoiding an additional rescaling step during integer inference, since a quantized bias would require its own scale and would need to be added to the accumulator after the dot product. 

With the bias removed, the linear layer reduces to a pure matrix multiplication:

$$
y_{n,m} = \sum_{k=0}^{K-1} x_{n,k}\, w_{k,m}
$$

To execute this operation using integer-only arithmetic, the inputs and weights are quantized and the matrix multiplication is reformulated so that the dot-product is computed with integers while the scale factors are applied separately.

First, the activations are stored in Q16 fixed-point format and quantized to the `int8` domain using a per-tensor scale factor stored in Q16 format:

$$
\hat{x}_{n,k} =
\operatorname{clip}\!\left(
\operatorname{round}\!\left(
\frac{x^{(Q16)}_{n,k}\, s^{(Q16)}_{x}}{2^{32}}
\right),
-127,\;127
\right)
$$

Where $$x^{(Q16)}_{n,k}$$ denotes the activation stored in Q16 fixed-point format at position $$(n,k)$$, and $$s^{(Q16)}_{x}$$ is the Q16 scale factor shared by all elements of the tensor. The multiplication of these two Q16 values produces an intermediate value in Q32 format, which is then rescaled to the integer domain before being clipped to the valid `int8` range.

Next, the weights are quantized offline using per-output-feature granularity,
where each output feature $$m$$ has its own scale factor:

$$
\hat{w}_{k,m} = \operatorname{round}(w_{k,m} \cdot s_m)
$$

Where $$w_{k,m}$$ denotes the floating-point weight connecting input feature $$k$$ to output feature $$m$$, and $$\hat{w}_{k,m}$$ is its quantized integer representation. The factor $$s_m$$ is the scale associated with output feature $$m$$ and is shared by all weights contributing to that output. 

The matrix multiplication can then be performed entirely with integer values:

$$
a_{n,m} = \sum_{k=0}^{K-1} \hat{x}_{n,k} \, \hat{w}_{k,m}
$$

Where $$a_{n,m}$$ denotes the integer accumulator resulting from the dot product between the quantized input and weight values contributing to output feature $$m$$.

Finally, the accumulator is rescaled back to the Q16 fixed-point domain using the inverse activation and weight scales:

$$
y^{(Q16)}_{n,m} \approx
\operatorname{round}\!\left(
\frac{
a_{n,m}\,
(s_x^{-1})^{(Q16)}\,
(s_m^{-1})^{(Q16)}
}{2^{16}}
\right)
$$

The factors $$(s_x^{-1})^{(Q16)}$$ and $$(s_m^{-1})^{(Q16)}$$ represent the inverse activation scale and the inverse weight scale associated with output feature $$m$$, both stored in Q16 fixed-point format. The division by $$2^{16}$$ restores the Q16 scaling after the fixed-point multiplications, producing the output activation $$y^{(Q16)}_{n,m}$$ in Q16 format.

This formulation allows the dot-product to be computed using integer multiplications and accumulations, while the scaling factors restore the appropriate numerical range for the following layer.


#### Convolution Operation for Convolutional Layers

In convolutional neural networks, the forward operation of a convolutional layer computes a weighted sum between a local region of the input tensor and a convolutional filter. For an output channel $$c$$ and spatial position $$(h,w)$$, the convolution can be written as:

$$
y_{n,c,h,w} =
\sum_{c_i=0}^{C_{\text{in}}-1}
\sum_{k_h=0}^{K_h-1}
\sum_{k_w=0}^{K_w-1}
x_{n,c_i,h',w'}\, w_{c,c_i,k_h,k_w}
$$

Where $$x_{n,c_i,h',w'}$$ denotes the input activation at batch index $$n$$, input channel $$c_i$$, and spatial position $$(h',w')$$ within the receptive field of the convolution. The term $$w_{c,c_i,k_h,k_w}$$ denotes the weight of the convolutional filter corresponding to output channel $$c$$, input channel $$c_i$$, and kernel coordinates $$(k_h,k_w)$$. The indices $$K_h$$ and $$K_w$$ denote the height and width of the convolution kernel, while $$C_{\text{in}}$$ represents the number of input channels.

As with the linear case, this operation can be reformulated to support integer-only inference by factoring out the quantization scale factors. To enable this factorization, the activations are quantized using per-tensor granularity, while the convolution weights are quantized using
per-channel granularity.

First, the input activations stored in Q16 fixed-point format are quantized to the `int8` domain using a single scale factor shared by the entire tensor:

$$
\hat{x}_{n,c_i,h,w} =
\operatorname{clip}\!\left(
\operatorname{round}\!\left(
\frac{x^{(Q16)}_{n,c_i,h,w}\, s_x^{(Q16)}}{2^{32}}
\right),
-127,\;127
\right)
$$

Where $$x^{(Q16)}_{n,c_i,h,w}$$ denotes the activation stored in Q16 fixed-point format and $$s_x^{(Q16)}$$ is the scale factor shared by all elements of the input tensor. The multiplication of these two Q16 values produces an intermediate value in Q32 format, which is then rescaled to the integer domain before being clipped to the valid `int8` range.

Next, the convolution weights are quantized offline using per-channel granularity, so that each output channel has its own scale factor:

$$
\hat{w}_{c,c_i,k_h,k_w}
=
\operatorname{round}
\left(
w_{c,c_i,k_h,k_w} \cdot s_c
\right)
$$

Where $$w_{c,c_i,k_h,k_w}$$ denotes the floating-point convolution weight and $$\hat{w}_{c,c_i,k_h,k_w}$$ is its quantized integer representation. The factor $$s_c$$ is the scale associated with output channel $$c$$ and is shared by all weights belonging to the same convolution filter.

The convolution can then be computed entirely using integer arithmetic:

$$
a_{n,c,h,w} =
\sum_{c_i=0}^{C_{\text{in}}-1}
\sum_{k_h=0}^{K_h-1}
\sum_{k_w=0}^{K_w-1}
\hat{x}_{n,c_i,h',w'}\, \hat{w}_{c,c_i,k_h,k_w}
$$

Where $$a_{n,c,h,w}$$ denotes the integer accumulator resulting from the dot product between the quantized input region and the quantized convolution filter associated with output channel $$c$$.

Finally, the accumulator is rescaled back to the Q16 fixed-point domain using the inverse activation and weight scales:

$$
y^{(Q16)}_{n,c,h,w}
\approx
\operatorname{round}\!\left(
\frac{
a_{n,c,h,w}\,
(s_x^{-1})^{(Q16)}\,
(s_c^{-1})^{(Q16)}
}{2^{16}}
\right)
$$

The factors $$(s_x^{-1})^{(Q16)}$$ and $$(s_c^{-1})^{(Q16)}$$ represent the inverse activation scale and the inverse weight scale associated with output channel $$c$$, both stored in Q16 fixed-point format. The division by $$2^{16}$$ restores the Q16 scaling after the fixed-point multiplications, producing the output activation $$y^{(Q16)}_{n,c,h,w}$$ in Q16 format.

As in the linear case, this formulation allows the convolution operation to be performed using integer multiplications and accumulations, while the scale factors restore the appropriate numerical range for the following layer.

#### Pooling Operation for Pooling Layers

Pooling layers reduce the spatial resolution of feature maps by applying a fixed reduction operation over local regions of the input tensor. In the case of max-pooling, the output activation corresponds to the maximum value within a spatial window.

Unlike linear and convolutional layers, pooling does not involve learnable parameters such as weights or biases. Instead, it performs a fixed reduction operation over the input region. As a result, no additional scale factors are introduced and the operation can be applied directly to the integer activations produced by the previous layer.

Since the activations entering the pooling layer are already represented in Q16 fixed-point format, the pooling operation can be applied directly without any additional quantization or rescaling. For an output channel $$c$$ and spatial position $$(h,w)$$, the operation can be written as:

$$
y^{(Q16)}_{n,c,h,w} =
\max_{\substack{0 \le k_h < K_h \\ 0 \le k_w < K_w}}
x^{(Q16)}_{n,c,h',w'}
$$

Where $$x^{(Q16)}_{n,c,h',w'}$$ denotes the input activation at batch index $$n$$, channel $$c$$, and spatial position $$(h',w')$$ within the pooling window, stored in Q16 fixed-point format. The indices $$K_h$$ and $$K_w$$ denote the height and width of the pooling kernel, while $$(h',w')$$ represents
the coordinates inside the receptive field corresponding to the output location $$(h,w)$$.

Because the operation only selects the maximum value within the local window, no multiplications or rescaling steps are required, and the Q16 numerical representation is preserved for the following layer.

#### Activation Functions in the Integer Domain

Activation functions introduce nonlinear transformations that allow neural networks to approximate complex mappings between inputs and outputs. In integer-only inference, these functions must operate directly on the fixed-point representations used by the network.

In the implementation presented here, activations produced by linear and convolutional layers are stored in Q16 fixed-point format. Among the available nonlinearities, the Rectified Linear Unit (ReLU) is particularly suitable because it can be applied directly to integer values. In Q16 format, the ReLU
operation is defined as:

$$
y^{(Q16)} = \max\left(0,\; x^{(Q16)}\right)
$$

Where $$x^{(Q16)}$$ denotes the input activation stored in Q16 format and $$y^{(Q16)}$$ is the resulting output activation. Since the operation only requires a comparison with zero, it can be implemented efficiently using integer arithmetic while preserving the Q16 representation.

