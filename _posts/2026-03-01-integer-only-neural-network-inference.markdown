---
title: "Integer-Only Neural Network Inference"
date: 2026-03-01 10:00:00
description: Quantizing a Convolutional Neural Network for digit recognition and implementing integer-only inference with fixed-point arithmetic.
author: lucasmazzetto
keywords: integer-only inference, quantized neural network, int8 quantization, fixed-point arithmetic, convolutional neural network, CNN, digit recognition, post-training quantization, calibration, embedded machine learning, computer vision
---

This project presents an integer-only neural network inference engine for handwritten digit recognition. A Convolutional Neural Network (CNN) was trained in PyTorch, quantized, and implemented in C using only integer arithmetic. The work includes calibration, model conversion, and evaluation against the original floating-point model. Finally, the model was deployed on an STM32 microcontroller connected to a camera. The quantized model maintained nearly the same classification accuracy while reducing parameter storage by almost a factor of four, demonstrating the feasibility of the approach for embedded applications.

## Introduction 

Microcontrollers are everywhere, from IoT sensors and small actuators to mobile robots that must run on a battery and react in real time. However, they usually have limited RAM, flash memory, and clock speed, and sometimes they don't even have a floating-point unit (FPU) for fast floating-point operations.

Meanwhile, neural networks are computationally expensive. They need many multiplications and a lot of memory bandwidth, and they usually store weights as `float32`. On a microcontroller, this is a problem because each `float32` value occupies 4 bytes and, without an FPU, floating-point operations can be slow and waste energy, which is exactly what you do not want in real-time and battery-powered systems.

Even though microcontrollers and neural networks may seem far apart, we can bring them closer with techniques like quantization. With `int8` quantization, we reduce the model size and speed up inference by replacing float weights with 8-bit integers, while keeping scale factors to preserve the original numeric range. In practice, this can shrink the footprint by four times and even make the model fast enough to run on a microcontroller.

This article presents a simple proof of concept. A Convolutional Neural Network (CNN) was trained for digit recognition, quantized from `float32` to `int8`, exported with quantized weights and scale constants, and executed on an STM32 microcontroller using a fixed-point inference engine. The entire source code for this project is available on [GitHub](https://github.com/workabotic/quantized_digit_recognition). You can explore it, reuse it in your own projects, and contribute if you want.

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

![Asymmetric Quantization Schema]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/asymmetric_quantization_schema.webp)
*Figure 1 — Asymmetric Quantization Schema.*

#### Symmetric Quantization


Now we can specialize the affine mapping to symmetric quantization by setting the zero-point to zero ($$z = 0$$):

$$
\hat{x} = \operatorname{clip}\left(\operatorname{round}(x \cdot s),\; -Q,\; Q\right)
$$

$$
x \approx \hat{x} \cdot s^{-1}
$$

In this formulation, the quantized integer values are restricted to the symmetric range $$[-Q, Q]$$, where $$Q$$ represents the maximum magnitude representable by the chosen integer format. To determine the value of $$Q$$, we need to consider how signed integers are represented. In the two's-complement convention, a signed integer type with $$b$$ bits has the representable range:

$$
[-2^{b-1},\; 2^{b-1}-1]
$$

However, symmetric quantization typically uses the slightly smaller interval so that the positive and negative ranges have the same magnitude:

$$
[-Q, Q], \qquad Q = (2^{b-1}-1)
$$

The most negative value $$-2^{b-1}$$ is avoided because it has no positive counterpart and would break the symmetry around zero. For example, with signed `int8` ($$b=8$$), the representable range is $$[-128, 127]$$. To maintain symmetry, we choose:

$$
Q = 2^{(8-1)}-1 = 127
$$

This yields the symmetric interval $$[-127, 127]$$.

Once the integer range is defined, symmetric quantization must determine a clipping threshold $$\alpha$$ that represents the range of real values to be mapped into this interval. The threshold plays the same role as the real-value bounds $$[r_{\min}, r_{\max}]$$ in asymmetric quantization, but with a symmetric interval $$[-\alpha,\alpha]$$. In the simplest case, $$\alpha = \max(\lvert r_{\min} \rvert, \lvert r_{\max} \rvert)$$, so that the whole observed range fits inside the symmetric interval.

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

![Symmetric Quantization Schema]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/symmetric_quantization_schema.webp)
*Figure 2 — Symmetric Quantization Schema.*

#### Quantization Granularity

Another important aspect of quantized neural networks is the granularity used when assigning scale factors. Granularity defines how many elements of a tensor share the same quantization parameters. 

The choice of granularity involves a trade-off between numerical accuracy and computational simplicity. Sharing the same quantization parameters across many elements simplifies the implementation and reduces computational cost, since fewer scale values must be stored and applied. However, this may increase quantization error because a single scale must represent values with different ranges. Using finer granularity allows scale parameters to better match the local distribution of values, reducing quantization error, but at the cost of additional parameters and more rescaling operations during inference.

In per-tensor quantization, a single scale parameter is used for all elements of a tensor. If $$x$$ is the input activation tensor, the quantized representation is therefore computed element-wise as:

$$
\hat{x} = \operatorname{clip}\left(\operatorname{round}(x \cdot s),\; -Q,\; Q\right)
$$

where $$s$$ is shared by every element of the tensor. The clipping step is important because rounding alone may produce values outside the target signed integer range. This approach is simple and efficient because the same scale factor can be applied to all values.

However, weight matrices often have different numerical ranges across their output columns. To better preserve these variations, each output column can be assigned its own scale factor.

$$
\hat{w}_{k,m} = \operatorname{clip}\left(\operatorname{round}(w_{k,m} \cdot s_m),\; -Q,\; Q\right)
$$

where $$m$$ identifies the output feature and $$k$$ identifies the input feature. Each column $$m$$ contains the weights connecting all input features to that output. Therefore, the same scale factor $$s_m$$ is applied to every weight in that column, while other columns can use different scales.

In convolutional layers, a similar strategy is used but the granularity is typically defined per output channel. Each convolution filter corresponds to one output channel, and different filters may have very different numerical ranges. Using a single scale for all filters could therefore introduce large quantization errors. In per-channel quantization, each output channel is assigned its own scale factor, allowing the quantization to better match the distribution of values within each filter while still keeping the computation efficient.

If $$w_{c,k,i,j}$$ denotes the weight of a convolution kernel for output channel $$c$$, input channel $$k$$, and spatial coordinates $$(i,j)$$, the quantized representation is computed element-wise as:

$$
\hat{w}_{c,k,i,j} = \operatorname{clip}\left(\operatorname{round}(w_{c,k,i,j} \cdot s_c),\; -Q,\; Q\right)
$$

where $$s_c$$ is the scale factor associated with output channel $$c$$, shared by all elements of the filter corresponding to that channel.


### Calibration

In PTQ, the scale parameters must be chosen so that the integer grid covers the range of values that actually appear during inference. During this process, a representative subset of the dataset is passed through the trained floating-point model and statistics of the activations are collected. These statistics are then used to estimate the clipping bound that defines the quantization scale.

Several strategies can be used to estimate this bound. The simplest is maximum value calibration, which uses the largest activation magnitude observed during calibration. To reduce the influence of rare extreme values, percentile calibration instead sets the bound at a high percentile of the observed magnitudes, allowing a small fraction of values to be clipped. For a more detailed view of these values, histogram-based calibration builds a histogram to approximate their probability distribution.

![Calibration Methods]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/calibration_methods.webp)
*Figure 3 — Calibration Methods (image extracted from [Wu et al. 2020](https://arxiv.org/abs/2004.09602)).*

#### Maximum value calibration

The simplest approach is maximum calibration, which sets the clipping bound equal to the largest observed absolute value:

$$
\alpha = \max |x|
$$

This method guarantees that all observed values are representable without clipping. Although simple, max calibration can be sensitive to outliers: a single large value may increase the clipping threshold and the quantization step size, reducing precision for most values.

#### Percentile calibration

Percentile calibration determines the clipping bound by selecting a high percentile of the observed activation magnitude distribution. Instead of using the maximum value, this method defines the clipping range so that a small fraction of extreme values is ignored. If $$p$$ denotes the chosen percentile, the clipping bound is defined as:

$$
\alpha = \text{percentile}(|x|, p)
$$

By discarding rare outlier values, percentile calibration allocates more of the available integer levels to the range where most activations occur. This often improves the effective quantization resolution compared to maximum calibration, especially when the activation distribution contains a few large but infrequent values.

#### Histogram-based calibration

Histogram-based calibration estimates the dynamic range of activations by approximating their statistical distribution. During calibration, a representative dataset is passed through the model and the values observed at the inputs of each quantized layer are collected. These values are used to build a histogram that approximates the probability distribution of the activations, denoted by $$p(x)$$.

Using this estimated distribution, the calibration step determines a suitable range parameter $$\alpha$$ that represents the magnitude of the activations to be preserved during quantization. Unlike maximum calibration, which uses only the largest observed value, histogram-based calibration uses the full distribution $$p(x)$$ to estimate a range that better reflects how the activations are typically distributed across the dataset.

### Integer-only Inference

After quantization and calibration, the model parameters are associated with scale factors and clipping ranges that define how real-valued activations and weights are represented in the integer domain.

However, to fully benefit from quantization on embedded hardware, the forward pass of the neural network must be executed using integer arithmetic only. This requires adapting the neural network operations so they can operate directly on quantized integers. In convolutional neural networks, this includes reformulating convolution operations, matrix multiplications from linear layers, pooling operations, and activation functions to work in the integer domain. In addition, the floating-point scaling parameters obtained during calibration must be converted to fixed-point representations so that the rescaling steps required during inference can also be performed using integer arithmetic.

In practice, the convolutional and linear layers follow a similar sequence during integer-only inference. To begin, the input activations stored in Q16 fixed-point format are quantized to the `int8` domain using the appropriate scale factor. Using these quantized values, the layer performs its multiplications and additions with integer arithmetic, storing the accumulated results in a wider integer format, typically 32 bits for `int8` products. Once these calculations are complete, the results are rescaled back to Q16 using the inverse scale factors, allowing them to be passed to the next layer.

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
x^{(Q_n)} + y^{(Q_n)} \approx (x+y)^{(Q_n)}
$$

Multiplication produces a result with doubled fractional precision:

$$
x^{(Q_n)} \cdot y^{(Q_n)} \approx (xy)^{(Q_{2n})}
$$

To restore the original format, the result must be shifted right by $$n$$ bits:

$$
(xy)^{(Q_n)} \approx
\operatorname{round}\!\left(
\frac{x^{(Q_n)} \cdot y^{(Q_n)}}{2^{n}}
\right)
$$

Using fixed-point representation allows real values to be encoded as integers while preserving fractional precision. In integer-only neural network inference, activations and intermediate results are stored in fixed-point format, as are the scale factors and other parameters obtained during quantization. By representing these values as fixed-point integers, all operations in the forward pass, such as multiplications, accumulations, and rescaling, can be implemented using integer arithmetic only, eliminating the need for floating-point computation while still approximating the original behavior of the network.

#### Matrix Multiplication for Linear Layers

In a linear layer, the general feedforward operation includes both a matrix
multiplication and a bias term:

$$
y_{n,m} = \sum_{k=0}^{K-1} x_{n,k}\, w_{k,m} + b_m
$$

where $$x_{n,k}$$ denotes the element of the input matrix at row $$n$$ and column $$k$$, and $$w_{k,m}$$ denotes the element of the weight matrix at row $$k$$ and column $$m$$. The index $$k$$ represents the shared dimension of the matrix multiplication, and the summation accumulates the products between the corresponding elements of the input and weight matrices. The term $$b_m$$ is the bias associated with output column $$m$$, and $$K$$ denotes the size of the shared dimension between the two matrices. The indices $$n$$ and $$m$$ respectively denote the row and column of the resulting output matrix.

In this project, the bias term was removed from all trainable layers to simplify the integer-only inference. This avoided the need to store and quantize bias values or add them to the layer outputs. With this simplification, the linear layer reduces to a pure matrix multiplication:

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

where $$x^{(Q16)}_{n,k}$$ denotes the activation stored in Q16 fixed-point format at position $$(n,k)$$, and $$s^{(Q16)}_{x}$$ is the Q16 scale factor shared by all elements of the tensor. The multiplication of these two Q16 values produces an intermediate value in Q32 format, which is then rescaled to the integer domain before being clipped to the valid `int8` range.

Next, the weights are quantized once on the computer using floating-point operations before deployment. Since the trained weights remain fixed during inference, their quantized values can be stored and reused. The inputs, in contrast, must be quantized during inference because they change with each sample. For the weights, per-column quantization assigns a separate scale factor to each output column $$m$$:

$$
\hat{w}_{k,m} =
\operatorname{clip}\left(
\operatorname{round}(w_{k,m} \cdot s_m),
\; -127,\; 127
\right)
$$

where $$w_{k,m}$$ denotes the original floating-point weight at row $$k$$ and column $$m$$ of the weight matrix, and $$\hat{w}_{k,m}$$ is its quantized integer representation. The factor $$s_m$$ is the scale assigned to column $$m$$ and is shared by all weights in that column. Since the implementation uses signed `int8` symmetric quantization, the rounded weights are clipped to the interval $$[-127, 127]$$.

The matrix multiplication can then be performed entirely with integer values:

$$
a_{n,m} = \sum_{k=0}^{K-1} \hat{x}_{n,k} \, \hat{w}_{k,m}
$$

where $$a_{n,m}$$ denotes the integer accumulator at output row $$n$$ and column $$m$$. It is obtained by multiplying the values in row $$n$$ of the quantized input matrix by the corresponding values in column $$m$$ of the quantized weight matrix and adding the products.

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

The factors $$(s_x^{-1})^{(Q16)}$$ and $$(s_m^{-1})^{(Q16)}$$ represent the inverse activation scale and the inverse weight scale assigned to column $$m$$ of the weight matrix, both stored in Q16 fixed-point format. Unlike the forward scales used to quantize values into the integer domain, these inverse scales restore the accumulator back to the Q16 domain. The division by $$2^{16}$$ restores the Q16 scaling after the fixed-point multiplications, producing the output activation $$y^{(Q16)}_{n,m}$$ in Q16 format.

Each product of two `int8` values fits in 16 bits. When adding these products, a 16-bit accumulator may also be sufficient, depending on the number of products and their possible values. If the sum can exceed its range, a wider accumulator, such as a 32-bit integer, is needed. Once the sum is complete, multiplying it by the two Q16 inverse scales may require an even wider intermediate, such as a 64-bit integer, to avoid overflow. Finally, the result can be rounded and shifted right by 16 bits to return to Q16 format.

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

where $$x_{n,c_i,h',w'}$$ denotes the input activation at batch index $$n$$, input channel $$c_i$$, and spatial position $$(h',w')$$ within the receptive field of the convolution. For stride 1 and no padding, the input coordinates are given by $$h' = h + k_h$$ and $$w' = w + k_w$$. For other stride or padding configurations, these indices are adjusted according to the corresponding convolution parameters. The term $$w_{c,c_i,k_h,k_w}$$ denotes the weight of the convolutional filter corresponding to output channel $$c$$, input channel $$c_i$$, and kernel coordinates $$(k_h,k_w)$$. The indices $$K_h$$ and $$K_w$$ denote the height and width of the convolution kernel, while $$C_{\text{in}}$$ represents the number of input channels.

As with linear layers, this operation can be reformulated to support integer-only inference by factoring out the quantization scale factors. To enable this factorization, the activations are quantized using per-tensor granularity, while the convolution weights are quantized using per-channel granularity.

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

where $$x^{(Q16)}_{n,c_i,h,w}$$ denotes the activation stored in Q16 fixed-point format and $$s_x^{(Q16)}$$ is the scale factor shared by all elements of the input tensor. The multiplication of these two Q16 values produces an intermediate value in Q32 format, which is then rescaled to the integer domain before being clipped to the valid `int8` range.

As with the linear layers, the convolution weights can be quantized offline on a computer using floating-point operations. This conversion is needed only once before deployment because the trained weights remain fixed during inference, allowing their quantized values to be stored and reused. Here, per-channel quantization assigns a separate scale factor to each output channel:

$$
\hat{w}_{c,c_i,k_h,k_w}
=
\operatorname{clip}\left(
\operatorname{round}
\left(
w_{c,c_i,k_h,k_w} \cdot s_c
\right),
\; -127,\; 127
\right)
$$

where $$w_{c,c_i,k_h,k_w}$$ denotes the floating-point convolution weight and $$\hat{w}_{c,c_i,k_h,k_w}$$ is its quantized integer representation. The factor $$s_c$$ is the scale associated with output channel $$c$$ and is shared by all weights belonging to the same convolution filter. As in the linear layers, the implementation uses signed `int8` symmetric quantization, so the rounded weights are clipped to $$[-127, 127]$$.

The convolution can then be computed entirely using integer arithmetic:

$$
a_{n,c,h,w} =
\sum_{c_i=0}^{C_{\text{in}}-1}
\sum_{k_h=0}^{K_h-1}
\sum_{k_w=0}^{K_w-1}
\hat{x}_{n,c_i,h',w'}\, \hat{w}_{c,c_i,k_h,k_w}
$$

where $$a_{n,c,h,w}$$ is the integer accumulator for output channel $$c$$ at position $$(h,w)$$ in sample $$n$$. It is computed by multiplying each quantized value in the input window by its matching filter weight and adding the products across all kernel positions and input channels.

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

The factors $$(s_x^{-1})^{(Q16)}$$ and $$(s_c^{-1})^{(Q16)}$$ represent the inverse activation scale and the inverse weight scale associated with output channel $$c$$, both stored in Q16 fixed-point format. The division by $$2^{16}$$ restores the Q16 scaling after the fixed-point multiplications, producing the output activation $$y^{(Q16)}_{n,c,h,w}$$ in Q16 format. As in the linear case, the accumulated sum can be stored in a 32-bit or 64-bit integer, depending on the number of products and their possible values. During rescaling, the intermediate must also be wide enough to hold the product of this sum and the two Q16 inverse scales without overflow. The result can then be rounded and shifted right by 16 bits to return to Q16.

This formulation allows the convolution operation to be performed using integer multiplications and accumulations, while the scale factors restore the appropriate numerical range for the following layer.

#### Pooling Operation for Pooling Layers

Pooling layers reduce the spatial resolution of feature maps by combining values within local windows. In the case of max-pooling, only the largest value in each window is kept. Since this requires only comparisons and introduces no learnable weights or biases, the operation can be applied directly to the Q16 activations from the previous layer. As a result, no multiplications, additional quantization, or rescaling are needed, and the output remains in Q16 format. With this representation, the operation for an output channel $$c$$ and spatial position $$(h,w)$$ can be written as:

$$
y^{(Q16)}_{n,c,h,w} =
\max_{\substack{0 \le k_h < K_h \\ 0 \le k_w < K_w}}
x^{(Q16)}_{n,c,h',w'}
$$

where $$x^{(Q16)}_{n,c,h',w'}$$ denotes the input activation at batch index $$n$$, channel $$c$$, and spatial position $$(h',w')$$ within the pooling window, stored in Q16 fixed-point format. The indices $$K_h$$ and $$K_w$$ denote the height and width of the pooling kernel, while $$(h',w')$$ represents the coordinates inside the receptive field corresponding to the output location $$(h,w)$$. For a pooling window applied with stride $$S$$ and no padding, $$h' = S\,h + k_h$$ and $$w' = S\,w + k_w$$.

#### Activation Functions in the Integer Domain

Activation functions introduce nonlinear transformations that allow neural networks to approximate complex mappings between inputs and outputs. In integer-only inference, these functions must operate directly on the fixed-point representations used by the network.

In the implementation presented here, activations produced by linear and convolutional layers are stored in Q16 fixed-point format. Among the available functions, the Rectified Linear Unit (ReLU) is particularly suitable because it can be applied directly to integer values. In Q16 format, the ReLU
operation is defined as:

$$
y^{(Q16)} = \max\left(0,\; x^{(Q16)}\right)
$$

where $$x^{(Q16)}$$ denotes the input activation stored in Q16 format and $$y^{(Q16)}$$ is the resulting output activation. Since the operation only requires a comparison with zero, it can be implemented efficiently using integer arithmetic while preserving the Q16 representation.

Figure 4 illustrates why this choice matters. With the symmetric quantization used here, ReLU keeps its original rule because zero is still represented by zero. Negative values are set to zero and the others pass through unchanged, so no function approximation or additional rescaling is needed. Sigmoid, however, depends on an exponential and a division. An integer-only implementation therefore needs an approximation, such as a lookup table or a polynomial, to produce its outputs. This adds code and can introduce further numerical error. ReLU was chosen for this project because it keeps the implementation simple and preserves the Q16 format.

![Comparison between ReLU and Sigmoid activation functions under quantization]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/relu_vs_sigmoid_quantization.webp)
*Figure 4 — Comparison between ReLU and Sigmoid activation functions in the context of quantization. ReLU is more suitable for integer-only inference due to its simplicity, while Sigmoid introduces additional complexity in quantized implementations (adapted from [Yousefzadeh and Sifalakis 2021](https://arxiv.org/abs/2107.07305)).*


## Requirements

For the development of this project, the following software dependencies were installed and used as part of the quantization pipeline:

- [**Linux Ubuntu 24.04**](https://ubuntu.com/blog/tag/ubuntu-24-04-lts) was used as the operating system.
- [**Python 3.10**](https://www.python.org/) was used as the programming language for training, quantization, conversion, and evaluation.
- [**PyTorch 2.9**](https://pytorch.org/) was used as the deep learning framework.
- [**GCC 13**](https://gcc.gnu.org/) was used as the C compiler for the integer-only inference engine.

After installing the operating system, the packages needed to compile the C inference engine and create an isolated Python environment were installed using the following command:

```bash
sudo apt install -y build-essential python3-venv
```

Then, a project directory named `quantized_digit_recognition` was created, together with a set of subdirectories to organize the scripts, the C sources, and the generated artifacts. The directory structure included the following components:

- `scripts`, which was used to store the Python scripts for training, quantization, parameter generation, and evaluation.
- `src` and `include`, which were used to store the C sources and headers of the inference engine, including the generated `params.c` and `params.h`.
- `models`, which was used to store the trained and quantized checkpoints.
- `data`, which was used to store the MNIST dataset downloaded by the scripts.
- `images`, which was used to store the training curves, confusion matrices, and feature-map plots.
- `lib`, which was used to store the compiled shared library.

These directories were created by executing the following commands:

```bash
mkdir -p ~/quantized_digit_recognition

cd ~/quantized_digit_recognition

mkdir -p scripts src include models data images lib
```

At this point, the `quantized_digit_recognition` project had the following directory structure:

```text
quantized_digit_recognition/
├── data/
├── images/
├── include/
├── lib/
├── models/
├── scripts/
└── src/
```

Next, a Python virtual environment named `.venv` was created inside the project root and activated, so that the Python dependencies could be installed in isolation from the system packages:

```bash
python3 -m venv .venv

source .venv/bin/activate

python -m pip install --upgrade pip
```

After activating the virtual environment, the required Python dependencies were installed using `pip`. These dependencies were used for handling the dataset, training the neural network, converting the parameters to fixed-point format, and generating the plots. The command presented below was used to install them:

```bash
pip install "numpy>=2" \
            "matplotlib>=3.8" \
            torch==2.9.1 \
            torchvision==0.24.1
```

With these steps completed, the environment setup was finalized, and the project was ready to proceed with the training and quantization of the neural network.

## Methods

Building on the quantization and fixed-point operations described earlier, the project began by training a LeNet-based CNN on the [MNIST](https://en.wikipedia.org/wiki/MNIST_database) dataset. Once trained, the model was quantized, and its parameters were exported for use in a C inference engine using only integer arithmetic. To assess the impact of this conversion on classification accuracy, the resulting implementation was compared against the original floating-point model. Finally, it was deployed on an STM32 microcontroller to evaluate its feasibility on embedded hardware. The following sections trace this process from model architecture through evaluation and deployment.

### Model Architecture

The model followed a [LeNet-style](http://vision.stanford.edu/cs598_spring07/papers/Lecun98.pdf) architecture, implemented in PyTorch as `ConvNet` in `scripts/model.py`. First, two convolutional blocks extracted visual patterns from a $$28 \times 28$$ grayscale image, while max-pooling reduced the size of the feature maps. The resulting maps were then flattened into 256 values and passed through three fully connected layers with 120, 84, and 10 outputs, producing one score for each digit class. ReLU was applied after each convolution and between the fully connected layers. This design kept the path from image pixels to digit scores simple, using operations that could all run with integer arithmetic.

To simplify integer-only inference, all convolutional and linear layers used `bias=False`, avoiding the need to store and quantize biases, as discussed in the [Matrix Multiplication for Linear Layers](#matrix-multiplication-for-linear-layers) section. Similarly, ReLU kept activation processing simple by requiring only a comparison with zero while preserving the fixed-point format. In addition, `_get_conv_output_size` computed the flattened feature count from a dummy input, allowing the convolutional and fully connected blocks to be connected without hard-coding the first linear layer's input size. The complete implementation, incorporating these choices, is shown below:

```python
import torch
import torch.nn as nn


class ConvNet(nn.Module):

    def __init__(self, h=28, w=28, inputs=1, outputs=10):
        """
        @brief Initialize the convolutional network architecture.

        @param h Input image height.
        @param w Input image width.
        @param inputs Number of input channels.
        @param outputs Number of output classes.
        """
        super(ConvNet, self).__init__()

        self.convolutional_layers = nn.Sequential(
            # First layer
            nn.Conv2d(inputs, 6, kernel_size=5, stride=1, bias=False),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
            
            # Second layer
            nn.Conv2d(6, 16, kernel_size=5, stride=1, bias=False),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2),
        )
        
        # Get the number of features produced by the convolutional block
        conv_output_size = self._get_conv_output_size(h, w, inputs)

        self.linear_layers = nn.Sequential(
            nn.Linear(in_features=conv_output_size, out_features=120, bias=False),
            nn.ReLU(),
            nn.Linear(in_features=120, out_features=84, bias=False),
            nn.ReLU(),
            nn.Linear(in_features=84, out_features=outputs, bias=False),
        )
    
    def _get_conv_output_size(self, h, w, inputs):
        """
        @brief Computes the number of output features produced by the convolutional layers.

        This method generates a dummy tensor with the same shape as the actual
        input images and feeds it through the convolutional base. By examining
        the resulting tensor size, it determines the exact number of features
        required by the first fully connected layer.

        @param h The input image height.
        @param w The input image width.
        @param inputs The number of input channels.
        @return The flattened feature size after all convolutional layers.
        """
        # Create a dummy tensor
        x = torch.zeros(1, inputs, h, w)
        
        # Pass dummy input through the convolutional layers
        x = self.convolutional_layers(x)
        
        # Get the total number of elements in x
        return x.numel()

    def forward(self, x):
        """
        @brief Run a forward pass through the network.

        @param x Input tensor with shape (N, C, H, W).
        @return Output logits tensor with shape (N, outputs).
        """
        x = self.convolutional_layers(x)
        x = x.view(x.size(0), -1)
        return self.linear_layers(x)
```

The trained model was exported to the ONNX format and opened in [Netron](https://github.com/lutzroeder/netron) to visualize its computational graph. This tool provides an interface for inspecting neural network architectures by displaying their structure, including the shape of each weight tensor and of the activations flowing between layers. The image below shows the graph representation of the model as rendered by the tool:

![Neural network architecture]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/neural_network_architecture.webp)
*Figure 5 — Neural network architecture.*

With the architecture defined, the next step was to train the network on the MNIST dataset.

### Neural Network Training

The training procedure was implemented in `scripts/train.py`. The MNIST training split was downloaded with `torchvision` and divided into training and validation subsets with `random_split`, using 80% of the samples for training and 20% for validation, while the MNIST test split was kept apart for the final evaluation.

To prepare the network for camera images, training and validation images were randomly rotated, shifted, resized, and distorted to represent differences in digit placement and handwriting. Changes in brightness and contrast, along with blur and noise, helped account for lighting and camera quality. Random inversion also exposed the model to both dark digits on light backgrounds and light digits on dark backgrounds. After these changes, normalization shifted and scaled the pixel values to give the model a consistent input scale centered on zero, mapping the input range to $$[-1, 1]$$. For testing, only grayscale conversion, random inversion, and the same normalization were used, so accuracy was measured on both background styles without the other image changes.

With the images prepared, the cross-entropy loss was used to measure how well the network's predictions matched the correct digit labels. The `Adam` optimizer then adjusted the network's weights to reduce this error, using its default learning rate to control the size of the updates. Data loaders were created with mini-batches of 128 samples, with shuffling enabled for the training and validation subsets. 

The script repeated training for the requested number of epochs, each representing one full pass through the training data. For each batch, it made predictions, measured the error, and updated the weights to reduce it. After each epoch, it checked the model on the validation images without changing the weights, then printed and plotted both loss values to track progress. Once training was complete, it measured accuracy on the separate MNIST test set and saved the trained weights, optimizer state, and completed epoch count in `models/model.pt`. The complete script is shown below:

```python
import os
import argparse
import multiprocessing
from pathlib import Path

import matplotlib
import torch
import torch.nn as nn
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from model import ConvNet
from torch.optim import Adam
from torch.utils.data import DataLoader, random_split

# Use a non-interactive backend so plots can be generated in headless environments
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class AddGaussianNoise:
    """
    @brief Adds zero-mean Gaussian noise to tensor images.

    @param std Standard deviation of the noise.
    """
    def __init__(self, std: float = 0.06):
        self.std = std

    def __call__(self, tensor: torch.Tensor):
        noisy = tensor + torch.randn_like(tensor) * self.std
        return torch.clamp(noisy, 0.0, 1.0)


def train_epoch(model:nn.Module, data_loader: DataLoader,
                optimizer: Adam, loss_fn:nn.CrossEntropyLoss, device: torch.device):
    """
    @brief Trains the model for one epoch.

    @param model The neural network model to train.
    @param data_loader The data loader containing the training data.
    @param optimizer The optimizer used to update model parameters.
    @param loss_fn The loss function used to compute the training loss.
    @return The average training loss for the epoch.
    """
    # Set the model to training mode
    model.train(mode=True)
    num_batches = len(data_loader)

    loss = 0
    
    for x, y in data_loader:
        x = x.to(device, non_blocking=True)
        y = y.to(device, non_blocking=True)

        # Reset gradients
        optimizer.zero_grad()
        
        # Forward pass
        outputs = model(x)

        batch_loss = loss_fn(outputs, y)

        # Backpropagation
        batch_loss.backward()
        
        # Update model parameters
        optimizer.step()

        loss += batch_loss.item()
        
    return loss / num_batches


def eval_epoch(model: nn.Module, data_loader: DataLoader,
               loss_fn: nn.CrossEntropyLoss, device: torch.device):
    """
    @brief Evaluates the model on the validation set.

    @param model The neural network model to evaluate.
    @param data_loader The data loader containing the validation data.
    @param loss_fn The loss function used to compute the validation loss.
    @return The average validation loss for the epoch.
    """
    # Set the model to evaluation mode
    model.eval()
    num_batches = len(data_loader)

    loss = 0
    
    # Disable gradient calculation
    with torch.no_grad():
        for x, y in data_loader:
            x = x.to(device, non_blocking=True)
            y = y.to(device, non_blocking=True)
            pred_y = model(x)
            batch_loss = loss_fn(pred_y, y)
            loss += batch_loss.item()
            
    return loss / num_batches


def train(model: nn.Module, train_loader: DataLoader, val_loader: DataLoader, 
          optimizer: Adam, loss_fn: nn.CrossEntropyLoss, num_epochs: int,
          plot_path: Path, device: torch.device):
    """
    @brief Runs the training loop for a specified number of epochs.

    @param model The neural network model to train.
    @param train_loader The data loader containing the training data.
    @param val_loader The data loader containing the validation data.
    @param optimizer The optimizer used to update model parameters.
    @param loss_fn The loss function used to compute the loss.
    @param num_epochs The number of epochs to train the model.
    @param plot_path Output path used to refresh the training loss plot every epoch.
    @return Lists containing train and validation loss per epoch.
    """
    print(f'Starting training for {num_epochs} epochs...')

    train_losses = []
    val_losses = []
    
    for epoch in range(num_epochs):
        train_loss = train_epoch(model, train_loader, optimizer, loss_fn, device)
        val_loss = eval_epoch(model, val_loader, loss_fn, device)
        train_losses.append(train_loss)
        val_losses.append(val_loss)

        # Update training.png at each epoch with the curve accumulated so far
        plot_training_loss(train_losses, val_losses, plot_path)

        print(f"Epoch [{epoch + 1}/{num_epochs}] - "
              f"Train Loss: {train_loss:.5f} | Validation Loss: {val_loss:.5f}")

    return train_losses, val_losses


def plot_training_loss(train_losses:list, val_losses:list, output_path: Path):
    """
    @brief Plots and saves the training/validation loss curves.

    @param train_losses List of train loss values per epoch.
    @param val_losses List of validation loss values per epoch.
    @param output_path Output image path for the loss plot.
    """
    epochs = range(1, len(train_losses) + 1)

    plt.figure(figsize=(8, 5))
    plt.plot(epochs, train_losses, label='Train loss', linewidth=2)
    plt.plot(epochs, val_losses, label='Validation loss', linewidth=2)
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.title('Training and Validation Loss')
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()


def test(model: nn.Module, test_loader: DataLoader, device: torch.device):
    """
    @brief Evaluates the model on the test dataset and prints accuracy.

    @param model The neural network model to evaluate.
    @param test_loader The data loader containing the test data.
    """
    print('Evaluating model performance on the test dataset...')
    
    model.eval()
    
    with torch.no_grad():
        acc = 0
        for samples, labels in test_loader:
            samples = samples.to(device, non_blocking=True)
            labels = labels.to(device, non_blocking=True)

            # Forward pass
            outputs = model(samples)
            
            # Compute probabilities
            probs = torch.nn.functional.softmax(outputs, dim=1)
            
            # Get predicted class
            preds = torch.argmax(probs, dim=1)
            
            # Accumulate correct predictions
            acc += (preds == labels).sum().item()

    print(f"Test Set Accuracy: {(acc / len(test_loader.dataset))*100.0:.3f}%")


def main(args):
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent
    images_dir = project_root / 'images'
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Training device: {device}")

    if device.type == "cuda":
        torch.backends.cudnn.benchmark = True

    # Create images directory to store training curves
    images_dir.mkdir(parents=True, exist_ok=True)

    # Augmentation for training and validation
    train_transform = transforms.Compose([transforms.Grayscale(),
                                          transforms.RandomRotation(30),
                                          transforms.RandomAffine(degrees=0, 
                                                                  translate=(0.25, 0.25), 
                                                                  scale=(0.5, 1.3), shear=10),
                                          transforms.RandomPerspective(distortion_scale=0.2, 
                                                                       p=0.5),
                                          transforms.ColorJitter(brightness=0.5, contrast=0.5),
                                          transforms.RandomInvert(p=0.5),
                                          transforms.ToTensor(),
                                          transforms.ElasticTransform(alpha=20.0, sigma=4.0),
                                          transforms.RandomApply([
                                              transforms.GaussianBlur(kernel_size=3,
                                                                      sigma=(0.1, 0.8))
                                          ], p=0.2),
                                          transforms.RandomApply([
                                              AddGaussianNoise(std=0.05)
                                          ], p=0.2),
                                          transforms.Normalize((0.5,), (0.5,))])

    # No augmentation for testing
    test_transform = transforms.Compose([transforms.Grayscale(),
                                         transforms.RandomInvert(p=0.5),
                                         transforms.ToTensor(),
                                         transforms.Normalize((0.5,), (0.5,))])

    # Download and load the MNIST training dataset
    mnist = datasets.MNIST(root=args.dataset_path, train=True,
                           download=True, transform=train_transform)
    
    # Split the dataset into training and validation sets
    train_dataset, val_dataset = random_split(mnist, [round(len(mnist) * args.train_split), 
                                                      round(len(mnist) * (1 - args.train_split))])

    # Download and load the MNIST test dataset
    test_dataset = datasets.MNIST(root=args.dataset_path, train=False, 
                                  download=True, transform=test_transform)

    # Initialize the Convolutional Neural Network
    model = ConvNet(h=28, w=28, inputs=1, outputs=10).to(device)

    # Set up the optimizer (Adam) and loss function (CrossEntropy)
    optimizer = Adam(model.parameters())
    loss_fn = nn.CrossEntropyLoss()

    # Create DataLoaders for batching and shuffling
    use_cuda = device.type == "cuda"

    train_loader = DataLoader(train_dataset, batch_size=args.batch_size,
                              num_workers=args.num_workers, shuffle=True,
                              pin_memory=use_cuda)
    
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size,
                            num_workers=args.num_workers, shuffle=True,
                            pin_memory=use_cuda)
    
    test_loader = DataLoader(test_dataset, batch_size=args.batch_size,
                             num_workers=args.num_workers, shuffle=False,
                             pin_memory=use_cuda)
    
    # Execute the training loop
    train_losses, val_losses = train(model, train_loader, val_loader, optimizer,
                                     loss_fn, args.num_epochs,
                                     images_dir / 'training.png', device)

    # Save training history figure in images/training.png
    plot_training_loss(train_losses, val_losses, images_dir / 'training.png')

    # Evaluate the model on the test set
    test(model, test_loader, device)

    os.makedirs(args.model_path, exist_ok=True)
    
    # Save the trained model, optimizer state, and epoch count
    torch.save({'state_dict': model.state_dict(),
                'optimizer_state': optimizer.state_dict(),
                'epoch': args.num_epochs},
                os.path.join(args.model_path, 'model.pt'))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Train a Convolutional Neural Network (CNN) on the MNIST dataset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument('--num_epochs', type=int, default=100,
                        help='Total number of training epochs to run.')
    
    parser.add_argument('--batch_size', type=int, default=128,
                        help='Mini-batch size for training, validation, and test loaders.')
    
    parser.add_argument('--train_split', type=float, default=0.8,
                        help='Fraction of MNIST train split used for training (rest for validation).')
    
    parser.add_argument('--dataset_path', type=Path, default='./data',
                        help='Directory where MNIST is downloaded/read.')
    
    parser.add_argument('--model_path', type=Path, default='./models',
                        help='Directory where the output model checkpoint is saved.')
    
    parser.add_argument('--num_workers', type=int, default=(multiprocessing.cpu_count() - 1),
                        help='Number of DataLoader worker subprocesses.')
    
    args = parser.parse_args()

    main(args)
```

The script was executed from the project root with the command shown below, training the model for 100 epochs:

```bash
python3 scripts/train.py --num_epochs 100 \
                         --batch_size 128 \
                         --train_split 0.8 \
                         --dataset_path ./data \
                         --model_path ./models
```

The training and validation losses recorded by the script at each epoch are shown in the image below:

![Training and validation loss]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/training_and_validation_loss.webp)
*Figure 6 — Training and validation loss.*

The resulting checkpoint was then used as the input of the quantization step described next.

### Post-Training Quantization and Calibration

After training, a script named `quantize.py` was created inside the `scripts` directory to apply post-training quantization to the model. It converted the trained weights to integers and prepared the scale factors needed by the C inference engine.

First, the script loaded `models/model.pt` and restored the trained `ConvNet`, which contained two convolutional layers and three linear layers. A calibration step was implemented to choose suitable input scales for these layers. During this step, the model was run on the MNIST training images, and the values entering each layer were measured to estimate their range. The trained weights remained unchanged during these measurements.

Three calibration methods were implemented: maximum value, percentile, and histogram-based calibration. Each method used the collected measurements to choose the input range represented by the quantized values. For this project, maximum value calibration was used, setting each layer's input range to the maximum absolute activation observed during calibration. The `--calibrator` argument allowed the calibration method to be selected for each run.

With the input ranges determined, the script converted the weights using symmetric quantization. Each output channel or feature received its own weight scale, while each layer used a single scale for its input values. The weights were scaled, rounded, and limited to the range from -127 to 127. The linear weight matrices were also transposed to match the layout expected by the C matrix multiplication.

Finally, the integer weights, input scales, and inverse input and weight scales were saved in `models/quantized.pt`. The scale factors were still stored as floating-point values at this stage and were converted to fixed-point integers in the next step. The complete script is shown below:

```python
import os
import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from torch.ao.quantization.observer import HistogramObserver
from torch.utils.data import DataLoader

from model import ConvNet


def get_quantized_layers(model: nn.Module):
    """
    @brief Returns the layers and metadata used by export.

    @param model Trained floating-point model.
    @return Ordered list of tuples with layer index, layer type, and module reference.
    """
    # Layer indices must match the checkpoint key pattern layer_1..layer_5
    return [(1, 'conv', model.convolutional_layers[0]),
            (2, 'conv', model.convolutional_layers[3]),
            (3, 'linear', model.linear_layers[0]),
            (4, 'linear', model.linear_layers[2]),
            (5, 'linear', model.linear_layers[4])]


def run_calibration_pass(model: nn.Module, data_loader: DataLoader,
                         hook_modules: dict, hook_callback):
    """
    @brief Runs one forward pass over calibration data with layer hooks.

    @param model Model used for calibration.
    @param data_loader Calibration data loader.
    @param hook_modules Mapping layer index -> module to hook.
    @param hook_callback Callback receiving layer index and input tensor.
    """
    handles = []

    for layer_idx, module in hook_modules.items():
        def pre_hook(_module, inputs, idx=layer_idx):
            # Capture layer input activations, which are what C quantize() uses
            hook_callback(idx, inputs[0].detach())

        handles.append(module.register_forward_pre_hook(pre_hook))

    model.eval()

    # Calibration only collects activation statistics, no gradients/updates
    with torch.no_grad():
        for batch, _ in data_loader:
            model(batch.float())

    for handle in handles:
        handle.remove()


def collect_amax_hist_observer(model: nn.Module, data_loader: DataLoader,
                               layer_modules: dict, num_bins: int):
    """
    @brief Collects activation amax values with PyTorch HistogramObserver.

    @param model Model used for calibration.
    @param data_loader Calibration data loader.
    @param layer_modules Mapping layer index -> module.
    @param num_bins Number of histogram bins.
    @return Mapping layer index -> activation amax.
    """
    observers = {}

    for layer_idx in layer_modules:
        observers[layer_idx] = HistogramObserver(dtype=torch.qint8,
                                                 qscheme=torch.per_tensor_symmetric,
                                                 quant_min=-127,
                                                 quant_max=127,
                                                 bins=num_bins)

    def hook_callback(layer_idx: int, tensor: torch.Tensor):
        observers[layer_idx](tensor.cpu())

    run_calibration_pass(model, data_loader, layer_modules, hook_callback)

    activation_amax = {}

    for layer_idx, observer in observers.items():
        scale, _ = observer.calculate_qparams()
        scale_value = float(scale.item())
        # Match C's [-127, 127] range and zero point 0: amax = scale * 127
        activation_amax[layer_idx] = max(scale_value * 127.0, 1e-12)

    return activation_amax


def collect_amax_max_value(model: nn.Module, data_loader: DataLoader,
                           layer_modules: dict):
    """
    @brief Collects activation amax values using max-value calibration.

    @param model Model used for calibration.
    @param data_loader Calibration data loader.
    @param layer_modules Mapping layer index -> module.
    @return Mapping layer index -> activation amax.
    """
    layer_max = {layer_idx: 0.0 for layer_idx in layer_modules}

    def collect_max(layer_idx: int, tensor: torch.Tensor):
        current_max = float(tensor.abs().max().item())
        if current_max > layer_max[layer_idx]:
            layer_max[layer_idx] = current_max

    run_calibration_pass(model, data_loader, layer_modules, collect_max)

    return {layer_idx: max(max_abs, 1e-12) for layer_idx, max_abs in layer_max.items()}


def compute_percentile_amax_from_histogram(histogram: np.ndarray, max_abs: float,
                                           percentile: float):
    """
    @brief Computes amax from an absolute-value histogram percentile threshold.

    @param histogram Absolute-value histogram counts.
    @param max_abs Maximum absolute value represented by the histogram.
    @param percentile Percentile in (0, 100].
    @return Estimated amax threshold.
    """
    if max_abs <= 0.0:
        return 1e-12

    hist = histogram.astype(np.float64)
    total = np.sum(hist)

    if total <= 0.0:
        return max_abs

    pct = float(np.clip(percentile, 1e-6, 100.0))
    target = (pct / 100.0) * total
    cdf = np.cumsum(hist)
    threshold_bin = int(np.searchsorted(cdf, target, side='left'))
    threshold_bin = min(max(threshold_bin, 0), hist.shape[0] - 1)

    # Convert selected bin index to its upper-edge value in [0, max_abs]
    return max_abs * (float(threshold_bin + 1) / float(hist.shape[0]))


def collect_amax_percentile(model: nn.Module, data_loader: DataLoader,
                            layer_modules: dict, num_bins: int, percentile: float):
    """
    @brief Collects activation amax values using percentile calibration.

    @param model Model used for calibration.
    @param data_loader Calibration data loader.
    @param layer_modules Mapping layer index -> module.
    @param num_bins Number of bins in calibration histograms.
    @param percentile Percentile used to choose clipping threshold.
    @return Mapping layer index -> activation amax.
    """
    # Collect max abs for stable histogram ranges
    layer_max = {layer_idx: 0.0 for layer_idx in layer_modules}

    def collect_max(layer_idx: int, tensor: torch.Tensor):
        current_max = float(tensor.abs().max().item())
        if current_max > layer_max[layer_idx]:
            layer_max[layer_idx] = current_max

    run_calibration_pass(model, data_loader, layer_modules, collect_max)

    # Collect absolute-value histograms with fixed [0, max_abs] ranges
    layer_hists = {layer_idx: np.zeros(num_bins, dtype=np.float64) for layer_idx in layer_modules}

    def collect_hist(layer_idx: int, tensor: torch.Tensor):
        max_abs = layer_max[layer_idx]
        if max_abs <= 0.0:
            return

        abs_tensor = tensor.abs().cpu().flatten()
        hist = torch.histc(abs_tensor, bins=num_bins, min=0.0, max=max_abs)
        layer_hists[layer_idx] += hist.numpy().astype(np.float64)

    run_calibration_pass(model, data_loader, layer_modules, collect_hist)

    activation_amax = {}

    for layer_idx in layer_modules:
        max_abs = layer_max[layer_idx]
        if max_abs <= 0.0:
            activation_amax[layer_idx] = 1e-12
            continue

        activation_amax[layer_idx] = max(compute_percentile_amax_from_histogram(
            layer_hists[layer_idx], max_abs, percentile), 1e-12)

    return activation_amax


def compute_weight_amax(weight: torch.Tensor):
    """
    @brief Computes per-output-channel amax for conv/linear weights.

    @param weight Weight tensor.
    @return Amax tensor broadcastable over weight dimensions.
    """
    # Per-output-channel amax matches current C layer scaling design
    reduce_dims = tuple(range(1, weight.dim()))
    return weight.abs().amax(dim=reduce_dims, keepdim=True).clamp(min=1e-12)


def quantize_model_params(model: nn.Module, activation_amax: dict):
    """
    @brief Quantizes model weights and exports fixed-point scaling constants.

    @param model Trained model.
    @param activation_amax Mapping layer index -> calibrated activation amax.
    @return Dictionary with quantized weights and scaling constants.
    """
    # int8 symmetric quantization range used by C implementation
    scale_factor = 127.0
    state_dict = dict()

    for layer_idx, layer_type, module in get_quantized_layers(model):
        weight = module.weight.detach().cpu()
        s_w = compute_weight_amax(weight)
        s_x = max(float(activation_amax[layer_idx]), 1e-12)

        scale = weight * (scale_factor / s_w)
        quantized_weight = torch.clamp(scale.round(), min=-127, max=127).to(torch.int32)

        if layer_type == 'linear':
            # Linear weights are transposed to match C mat_mult memory layout
            quantized_weight = quantized_weight.T

        # Export tensors and scale constants with existing checkpoint keys
        state_dict[f'layer_{layer_idx}_weight'] = quantized_weight.numpy()
        state_dict[f'layer_{layer_idx}_s_x'] = scale_factor / s_x
        state_dict[f'layer_{layer_idx}_s_x_inv'] = s_x / scale_factor
        state_dict[f'layer_{layer_idx}_s_w_inv'] = (s_w / scale_factor).squeeze().numpy()

    return state_dict


def build_calibration_loader(data_dir: Path):
    """
    @brief Builds MNIST calibration data loader.

    @param data_dir Directory containing MNIST data.
    @return DataLoader for calibration.
    """
    # Keep preprocessing aligned with train/eval scripts
    dataset = datasets.MNIST(root=data_dir,
                             train=True,
                             download=True,
                             transform=transforms.Compose([transforms.Grayscale(),
                                                           transforms.ToTensor(),
                                                           transforms.Normalize((0.5,), (0.5,))]))

    # Full-dataset single batch reproduces the original calibration workflow
    return DataLoader(dataset, batch_size=len(dataset), num_workers=1, shuffle=False)


def calibrate_activation_amax(model: nn.Module, data_loader: DataLoader,
                              calibrator: str, num_bins: int, percentile: float):
    """
    @brief Calibrates activation amax values with a selected calibrator.

    @param model Model used for calibration.
    @param data_loader Calibration data loader.
    @param calibrator Selected calibrator method.
    @param num_bins Number of histogram bins.
    @param percentile Percentile used by percentile calibrator.
    @return Mapping layer index -> activation amax.
    """
    layer_modules = {}

    for layer_idx, _, module in get_quantized_layers(model):
        layer_modules[layer_idx] = module

    # Dispatch to selected calibration strategy
    if calibrator == 'histogram_observer':
        return collect_amax_hist_observer(model, data_loader, layer_modules, num_bins)

    if calibrator == 'max_value':
        return collect_amax_max_value(model, data_loader, layer_modules)

    if calibrator == 'percentile':
        return collect_amax_percentile(model, data_loader, layer_modules, num_bins, percentile)

    raise ValueError(f'Unsupported calibrator: {calibrator}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Script for post-training quantization of a pre-trained model',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('--filename', type=str, default='model.pt',
                        help='Trained checkpoint filename inside --save_dir.')
    
    parser.add_argument('--num_bins', type=int, default=128,
                        help='Number of bins used for histogram-based calibration.')
    
    parser.add_argument('--calibrator', type=str,
                        choices=['histogram_observer', 'max_value', 'percentile'],
                        default='histogram_observer',
                        help='Calibration strategy used to estimate activation amax.')

    parser.add_argument('--percentile', type=float, default=99.9,
                        help='Percentile used when --calibrator=percentile.')
    
    parser.add_argument('--data_dir', default='./data', type=Path,
                        help='Directory where MNIST is downloaded/read for calibration.')
    
    parser.add_argument('--save_dir', default='./models', type=Path,
                        help='Directory containing input checkpoint and output quantized file.')

    args = parser.parse_args()

    # Load trained floating-point checkpoint
    saved_stats = torch.load(os.path.join(args.save_dir, args.filename), map_location="cpu")
    state_dict = saved_stats['state_dict']

    model = ConvNet(h=28, w=28, inputs=1, outputs=10)
    model.load_state_dict(state_dict)
    model.eval()

    # Calibrate activation amax values and export quantized parameters
    calibration_loader = build_calibration_loader(args.data_dir)

    activation_amax = calibrate_activation_amax(model, calibration_loader,
                                                args.calibrator, args.num_bins,
                                                args.percentile)

    quantized_state_dict = quantize_model_params(model, activation_amax)
    saved_stats['state_dict'] = quantized_state_dict

    torch.save(saved_stats, os.path.join(args.save_dir, 'quantized.pt'))
```

For the quantized model used in this project, maximum value calibration was selected for the activation ranges. To generate this version, the command below was run from the project root. The `--calibrator max_value` argument selected maximum value calibration. The remaining arguments kept their default values, reading the trained model from `models/model.pt` and saving the quantized parameters in `models/quantized.pt`:

```bash
python3 scripts/quantize.py --filename model.pt \
                            --calibrator max_value \
                            --num_bins 128 \
                            --save_dir ./models \
                            --data_dir ./data
```

At this point, `models/quantized.pt` contained the weights and floating-point scale factors, and it still had to be turned into something a C compiler could consume. The next section describes how `scripts/generate_params.py` converted this checkpoint into the `params.h` and `params.c` files used by the inference engine.

### Model Conversion

To convert the quantized model into C constants, a script named `generate_params.py` was created inside the `scripts` directory. It read `models/quantized.pt` and generated the files needed to include the model parameters in the C inference engine.

First, the script loaded the checkpoint and checked that the weights and scale factors were available for all five layers. It then calculated the layer dimensions using the saved weight arrays and the input image size provided through command-line arguments. These dimensions were also checked to ensure that the output of the convolutional block matched the input size of the first linear layer.

Finally, the script generated two files: `include/params.h`, which declared the network dimensions and parameters, and `src/params.c`, which stored their values. The weights were exported as `int8_t` arrays, while the floating-point scale factors were converted to Q16 integers by multiplying them by $$2^{16}$$ and rounding. This allowed the parameters to be compiled directly into the application and used during integer-only inference. The complete script is shown below:

```python
import argparse
import re
from pathlib import Path

import numpy as np
import torch

LAYER_PREFIX_BY_INDEX = {
    1: "conv_1",
    2: "conv_2",
    3: "linear_1",
    4: "linear_2",
    5: "linear_3",
}


def conv_output_dim(input_dim: int, kernel_size: int, stride: int):
    """
    @brief Calculate 1D output dimension for valid convolution/pooling.

    @param input_dim Input dimension.
    @param kernel_size Kernel size.
    @param stride Stride value.
    @return Output dimension.
    """
    return (input_dim - kernel_size) // stride + 1


def to_numpy(value):
    """
    @brief Convert tensors/arrays/scalars to numpy arrays.

    @param value Input value.
    @return NumPy representation of the input.
    """
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        return value
    return np.asarray(value)


def scalar_to_fixed_point(value, frac_bits: int):
    """
    @brief Convert scalar value to fixed-point integer.

    @param value Scalar value.
    @param frac_bits Number of fractional bits.
    @return Fixed-point integer value.
    """
    return int(np.rint(float(np.asarray(value)) * (1 << frac_bits)))


def array_to_fixed_point(values, frac_bits: int):
    """
    @brief Convert array values to fixed-point integer array.

    @param values Array-like input.
    @param frac_bits Number of fractional bits.
    @return Fixed-point NumPy integer array.
    """
    arr = np.atleast_1d(to_numpy(values)).astype(np.float64)
    return np.rint(arr * (1 << frac_bits)).astype(np.int64)


def get_layer_indices(state_dict):
    """
    @brief Get sorted layer indices from quantized state dictionary keys.

    @param state_dict Quantized state dictionary.
    @return Sorted list of layer indices.
    """
    indices = set()
    pattern = re.compile(r'^layer_(\d+)_weight$')
    for key in state_dict:
        match = pattern.match(key)
        if match:
            indices.add(int(match.group(1)))
    return sorted(indices)


def get_layer_prefix(layer_idx: int) -> str:
    """
    @brief Returns the generated C symbol prefix for a given layer index.

    @param layer_idx Layer index from the quantized checkpoint.
    @return C symbol prefix used in params.h/params.c.
    """
    if layer_idx not in LAYER_PREFIX_BY_INDEX:
        raise KeyError(f"Unsupported layer index for export symbols: {layer_idx}")

    return LAYER_PREFIX_BY_INDEX[layer_idx]


def infer_dimensions(state_dict, input_h: int, input_w: int, input_c: int):
    """
    @brief Infer all model dimensions from quantized weights and input shape.

    @param state_dict Quantized state dictionary.
    @param input_h Input height.
    @param input_w Input width.
    @param input_c Input channels.
    @return Dictionary with inferred dimensions and sizes.
    """
    w1 = to_numpy(state_dict['layer_1_weight'])
    w2 = to_numpy(state_dict['layer_2_weight'])
    w3 = to_numpy(state_dict['layer_3_weight'])
    w4 = to_numpy(state_dict['layer_4_weight'])
    w5 = to_numpy(state_dict['layer_5_weight'])

    c1 = int(w1.shape[0])
    c2 = int(w2.shape[0])
    k1 = int(w1.shape[2])
    k2 = int(w2.shape[2])

    h1_conv = conv_output_dim(input_h, k1, stride=1)
    w1_conv = conv_output_dim(input_w, k1, stride=1)
    h1_pool = conv_output_dim(h1_conv, kernel_size=2, stride=2)
    w1_pool = conv_output_dim(w1_conv, kernel_size=2, stride=2)

    h2_conv = conv_output_dim(h1_pool, k2, stride=1)
    w2_conv = conv_output_dim(w1_pool, k2, stride=1)
    h2_pool = conv_output_dim(h2_conv, kernel_size=2, stride=2)
    w2_pool = conv_output_dim(w2_conv, kernel_size=2, stride=2)

    linear_input = int(w3.shape[0])
    expected_linear_input = c2 * h2_pool * w2_pool
    if linear_input != expected_linear_input:
        raise ValueError(
            f'Mismatch in first linear input size: got {linear_input}, '
            f'expected {expected_linear_input} from conv dims.'
        )

    s1_linear = int(w3.shape[1])
    s2_linear = int(w4.shape[1])
    output_dim = int(w5.shape[1])

    return {'INPUT_HEIGHT': input_h,
            'INPUT_WIDTH': input_w,
            'INPUT_CHANNELS': input_c,
            'CONV_1_OUT_HEIGHT': h1_conv,
            'CONV_1_OUT_WIDTH': w1_conv,
            'POOL_1_OUT_HEIGHT': h1_pool,
            'POOL_1_OUT_WIDTH': w1_pool,
            'CONV_2_OUT_HEIGHT': h2_conv,
            'CONV_2_OUT_WIDTH': w2_conv,
            'POOL_2_OUT_HEIGHT': h2_pool,
            'POOL_2_OUT_WIDTH': w2_pool,
            'CONV_1_OUT_CHANNELS': c1,
            'CONV_2_OUT_CHANNELS': c2,
            'LINEAR_1_OUT_FEATURES': s1_linear,
            'LINEAR_2_OUT_FEATURES': s2_linear,
            'OUTPUT_DIM': output_dim}


def write_header_file(path: Path, state_dict, layer_indices, dims: dict):
    """
    @brief Write generated params.h file.

    @param path Output header path.
    @param state_dict Quantized state dictionary.
    @param layer_indices Sorted layer indices.
    @param dims Inferred dimensions dictionary.
    """
    with path.open('w', encoding='utf-8') as f:
        f.write('#ifndef PARAMS\n#define PARAMS\n\n')

        f.write(
            f"#define INPUT_FLAT_SIZE {dims['INPUT_HEIGHT'] * dims['INPUT_WIDTH'] * dims['INPUT_CHANNELS']}\n"
        )
        f.write(f"#define INPUT_HEIGHT {dims['INPUT_HEIGHT']}\n")
        f.write(f"#define INPUT_WIDTH {dims['INPUT_WIDTH']}\n")
        f.write(f"#define CONV_1_OUT_HEIGHT {dims['CONV_1_OUT_HEIGHT']}\n")
        f.write(f"#define CONV_1_OUT_WIDTH {dims['CONV_1_OUT_WIDTH']}\n")
        f.write(f"#define POOL_1_OUT_HEIGHT {dims['POOL_1_OUT_HEIGHT']}\n")
        f.write(f"#define POOL_1_OUT_WIDTH {dims['POOL_1_OUT_WIDTH']}\n")
        f.write(f"#define CONV_2_OUT_HEIGHT {dims['CONV_2_OUT_HEIGHT']}\n")
        f.write(f"#define CONV_2_OUT_WIDTH {dims['CONV_2_OUT_WIDTH']}\n")
        f.write(f"#define POOL_2_OUT_HEIGHT {dims['POOL_2_OUT_HEIGHT']}\n")
        f.write(f"#define POOL_2_OUT_WIDTH {dims['POOL_2_OUT_WIDTH']}\n")
        f.write(f"#define INPUT_CHANNELS {dims['INPUT_CHANNELS']}\n")
        f.write(f"#define CONV_1_OUT_CHANNELS {dims['CONV_1_OUT_CHANNELS']}\n")
        f.write(f"#define CONV_2_OUT_CHANNELS {dims['CONV_2_OUT_CHANNELS']}\n")
        f.write(f"#define LINEAR_1_OUT_FEATURES {dims['LINEAR_1_OUT_FEATURES']}\n")
        f.write(f"#define LINEAR_2_OUT_FEATURES {dims['LINEAR_2_OUT_FEATURES']}\n")
        f.write(f"#define OUTPUT_DIM {dims['OUTPUT_DIM']}\n\n")
        f.write('#include <stdint.h>\n\n')

        f.write('// quantization/dequantization constants\n')
        for layer_idx in layer_indices:
            layer_prefix = get_layer_prefix(layer_idx)

            f.write(f'extern const int {layer_prefix}_input_scale;\n')
            f.write(f'extern const int {layer_prefix}_input_scale_inv;\n')

            sw_inv = np.atleast_1d(to_numpy(state_dict[f'layer_{layer_idx}_s_w_inv']))
            f.write(f'extern const int {layer_prefix}_weight_scale_inv[{sw_inv.size}];\n')

        f.write('\n')
        f.write('// layer quantized parameters\n')
        for layer_idx in layer_indices:
            layer_prefix = get_layer_prefix(layer_idx)
            weight = to_numpy(state_dict[f'layer_{layer_idx}_weight'])
            f.write(f'extern const int8_t {layer_prefix}_weight[{weight.size}];\n')

        f.write('\n#endif // PARAMS\n')


def write_source_file(path: Path, state_dict, layer_indices, frac_bits: int):
    """
    @brief Write generated params.c file.

    @param path Output source path.
    @param state_dict Quantized state dictionary.
    @param layer_indices Sorted layer indices.
    @param frac_bits Number of fractional bits for fixed-point conversion.
    """
    with path.open('w', encoding='utf-8') as f:
        f.write('#include "params.h"\n\n')

        for layer_idx in layer_indices:
            layer_prefix = get_layer_prefix(layer_idx)
            sx = scalar_to_fixed_point(state_dict[f'layer_{layer_idx}_s_x'], frac_bits)
            sx_inv = scalar_to_fixed_point(state_dict[f'layer_{layer_idx}_s_x_inv'], frac_bits)
            sw_inv = array_to_fixed_point(state_dict[f'layer_{layer_idx}_s_w_inv'], frac_bits)

            f.write(f'const int {layer_prefix}_input_scale = {sx};\n\n')
            f.write(f'const int {layer_prefix}_input_scale_inv = {sx_inv};\n\n')

            sw_values = ', '.join(str(int(v)) for v in sw_inv.flatten())
            f.write(
                f'const int {layer_prefix}_weight_scale_inv[{sw_inv.size}] = '
                f'{{{sw_values}}};\n\n'
            )

        for layer_idx in layer_indices:
            layer_prefix = get_layer_prefix(layer_idx)
            weights = to_numpy(state_dict[f'layer_{layer_idx}_weight']).flatten()
            values = ', '.join(str(int(v)) for v in weights)
            f.write(
                f'const int8_t {layer_prefix}_weight[{weights.size}] = '
                f'{{{values}}};\n\n'
            )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Generate params.h and params.c from a quantized model checkpoint.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--path', type=Path, default=Path('models/quantized.pt'),
                        help='Path to quantized checkpoint to convert (e.g., quantized.pt).')
    parser.add_argument('--output_dir', type=Path, default=Path('src'),
                        help='Directory where generated params.c is written.')
    parser.add_argument('--header_dir', type=Path, default=Path('include'),
                        help='Directory where generated params.h is written.')
    parser.add_argument('--input_h', type=int, default=28,
                        help='Input image height used to infer network dimensions.')
    parser.add_argument('--input_w', type=int, default=28,
                        help='Input image width used to infer network dimensions.')
    parser.add_argument('--input_c', type=int, default=1,
                        help='Input channel count used to infer network dimensions.')
    parser.add_argument('--frac_bits', type=int, default=16,
                        help='Number of fractional bits for fixed-point scale conversion.')
    args = parser.parse_args()

    checkpoint = torch.load(args.path, map_location="cpu", weights_only=False)
    if 'state_dict' not in checkpoint:
        raise KeyError(f"'state_dict' not found in checkpoint: {args.path}")
    state_dict = checkpoint['state_dict']

    layer_indices = get_layer_indices(state_dict)
    if layer_indices != [1, 2, 3, 4, 5]:
        raise ValueError(
            f'Expected quantized layers [1, 2, 3, 4, 5], got {layer_indices}.'
        )

    required_suffixes = ('_weight', '_s_x', '_s_x_inv', '_s_w_inv')
    for layer_idx in layer_indices:
        for suffix in required_suffixes:
            key = f'layer_{layer_idx}{suffix}'
            if key not in state_dict:
                raise KeyError(f'Missing key in quantized state_dict: {key}')

    dims = infer_dimensions(state_dict, args.input_h, args.input_w, args.input_c)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.header_dir.mkdir(parents=True, exist_ok=True)
    header_path = args.header_dir / 'params.h'
    source_path = args.output_dir / 'params.c'

    write_header_file(header_path, state_dict, layer_indices, dims)
    write_source_file(source_path, state_dict, layer_indices, args.frac_bits)

    print(f'Wrote {header_path}')
    print(f'Wrote {source_path}')
```

It was executed from the project root with the command shown below:

```bash
python3 scripts/generate_params.py --path models/quantized.pt --output_dir src --header_dir include --frac_bits 16
```

The generated `params.h` file contained the network dimensions as `#define` constants and declared the weights and scale factors used by the C engine. These declarations specified the parameter names, types, and array sizes, while the actual values were defined in `params.c`. By including this header, the inference code could access the model parameters and use the dimensions to reserve space for intermediate results. The generated header is shown below:

```c
#ifndef PARAMS
#define PARAMS

#define INPUT_FLAT_SIZE 784
#define INPUT_HEIGHT 28
#define INPUT_WIDTH 28
#define CONV_1_OUT_HEIGHT 24
#define CONV_1_OUT_WIDTH 24
#define POOL_1_OUT_HEIGHT 12
#define POOL_1_OUT_WIDTH 12
#define CONV_2_OUT_HEIGHT 8
#define CONV_2_OUT_WIDTH 8
#define POOL_2_OUT_HEIGHT 4
#define POOL_2_OUT_WIDTH 4
#define INPUT_CHANNELS 1
#define CONV_1_OUT_CHANNELS 6
#define CONV_2_OUT_CHANNELS 16
#define LINEAR_1_OUT_FEATURES 120
#define LINEAR_2_OUT_FEATURES 84
#define OUTPUT_DIM 10

#include <stdint.h>

// quantization/dequantization constants
extern const int conv_1_input_scale;
extern const int conv_1_input_scale_inv;
extern const int conv_1_weight_scale_inv[6];
extern const int conv_2_input_scale;
extern const int conv_2_input_scale_inv;
extern const int conv_2_weight_scale_inv[16];
extern const int linear_1_input_scale;
extern const int linear_1_input_scale_inv;
extern const int linear_1_weight_scale_inv[120];
extern const int linear_2_input_scale;
extern const int linear_2_input_scale_inv;
extern const int linear_2_weight_scale_inv[84];
extern const int linear_3_input_scale;
extern const int linear_3_input_scale_inv;
extern const int linear_3_weight_scale_inv[10];

// layer quantized parameters
extern const int8_t conv_1_weight[150];
extern const int8_t conv_2_weight[2400];
extern const int8_t linear_1_weight[30720];
extern const int8_t linear_2_weight[10080];
extern const int8_t linear_3_weight[840];

#endif // PARAMS
```

The `params.c` file stored the actual weights and scale factors declared in `params.h`. These values came from the quantized checkpoint, with the weights stored as `int8_t` arrays and the scale factors converted to Q16 integers. With `params.h` and `params.c` generated, the model parameters were ready to be compiled with the C inference code. The next section explains how that code used them to run the network.

### Integer-Only Inference Engine

To run the quantized network using integer arithmetic only, an inference engine was implemented in C with the exported model parameters. The layer operations were placed in `src/nn.c`, with their declarations in `include/nn.h`. A separate file, `src/convnet.c`, connected these operations in the order required by the network. This kept the layer calculations separate from the code that defined the network sequence.

The engine used Q16 as the common format for values passed between layers. These activations were stored as 32-bit `int` values, while the weights and the quantized copies of the layer inputs used `int8_t`. The `FRAC_BITS` constant was set to 16 to match the scale factors exported during model conversion. This kept all layers working with the same fixed-point representation.

Each convolutional or linear layer started by converting its Q16 input to the `int8` range through the `quantize` function. Each input value was multiplied by the layer's input scale, which was also stored in Q16. Since both values had 16 fractional bits, their product had 32 fractional bits. An `int64_t` variable was used to hold this product because it could exceed the range of a 32-bit integer. A rounded right shift by 32 bits then removed the fractional scaling, effectively dividing by $$2^{32}$$. The result was limited to -127 through 127 before being stored as `int8_t`, preventing values outside the chosen range from wrapping around. The rounding helper worked on the magnitude of each value and restored its sign afterward, so positive and negative values were rounded consistently.

With the inputs quantized, `mat_mult` calculated each linear-layer output by multiplying every input value by the weight connecting it to that output and adding all the products. The loops repeated this calculation for every output feature. The `conv2d` function performed a similar calculation over a small window of the input. It multiplied the values in that window by the corresponding filter weights and added the products across the window and all input channels. This process was repeated at each window position and for every filter to produce the output feature maps. Both functions stored their sums in 32-bit accumulators because the products and their accumulated sum could exceed the range of an 8-bit integer.

These sums still reflected the scaling applied when the inputs and weights were quantized. To approximate the original layer outputs, each sum had to be multiplied by the inverse input and weight scales. All terms contributing to a particular output shared the same input scale and the same weight scale, so this correction could be applied once after the sum was complete. This kept the inner loops focused on integer multiplication and addition, without repeating the scale correction for every product.

The correction was implemented in `dequantize_per_channel` for convolutions and `dequantize_per_col` for linear layers. Each completed sum was multiplied by the layer's inverse input scale and the inverse weight scale for its output channel or feature. The intermediate products used `int64_t` variables because they could exceed the 32-bit range. Since the two inverse scales were stored in Q16, their product introduced 32 fractional bits. A rounded right shift by 16 bits reduced this to the 16 fractional bits required for Q16. The result was then stored as a 32-bit integer, ready for the following operations.

With the outputs back in Q16 format, the remaining operations could be performed using simple comparisons. ReLU compared each value with zero and kept the larger one. Similarly, max-pooling compared the values in each window and kept the largest one. Since both operations relied only on comparisons, they preserved the Q16 format without additional scaling. Once the final layer had produced its output values, `argmax_per_row` selected the index of the largest one as the predicted digit.

Finally, these operations were combined into the `linear_layer` and `conv2d_layer` functions. Each function quantized its input, computed the weighted sums, and rescaled the output. ReLU was always applied after convolution and could be enabled or disabled for a linear layer, allowing the final layer to keep its signed output values. Temporary quantized inputs were stored in arrays on the stack, avoiding heap allocation. 

The `include/nn.h` header is shown below:

```c
#ifndef NN_H
#define NN_H

#include <stdint.h>

#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

#define NUM_BITS 8
#define INT8_MAX_VALUE 127
#define FRAC_BITS 16

#define ROUND_CONST (1 << (FRAC_BITS - 1))    // = 0.5 before right shifting to improve rounding

/**
 * @brief Performs 2D max pooling.
 *
 * @param input The input tensor.
 * @param output The output tensor.
 * @param batch_size The number of samples.
 * @param channels The number of channels (pooling preserves channel count).
 * @param input_height The input height.
 * @param input_width The input width.
 * @param output_height The output height.
 * @param output_width The output width.
 * @param kernel_height The kernel height.
 * @param kernel_width The kernel width.
 * @param stride_height The height stride.
 * @param stride_width The width stride.
 */
void pooling2d(const int *input, int *output, int batch_size, int channels, int input_height,
               int input_width, int output_height, int output_width, int kernel_height,
               int kernel_width, int stride_height, int stride_width);

/**
 * @brief Computes argmax along columns for each row of a flattened matrix.
 *
 * Interprets `input` as row-major [batch_size, output_features]. For each row `n`, stores in
 * `indices[n]` the column index `m` with the maximum value.
 * If multiple columns share the maximum, the first one is kept.
 *
 * @param input Input matrix flattened in row-major order.
 * @param indices Output array of length `batch_size` with argmax column indices.
 * @param batch_size Number of matrix rows.
 * @param output_features Number of columns per row. Must be greater than zero.
 */
void argmax_per_row(const int *input, unsigned int *indices, const unsigned int batch_size,
                    const unsigned int output_features);

/**
 * @brief Runs a linear neural-network layer without bias.
 *
 * Input is quantized before multiplication with weights and then dequantized per-column before
 * optional activation.
 *
 * @param input The [batch_size x input_features] input matrix.
 * @param weights The [input_features x output_features] layer weight matrix.
 * @param output The [batch_size x output_features] output matrix.
 * @param input_scale The scale factor for input quantization.
 * @param weight_scale_inv The [1 x output_features] inverse scale vector for weights.
 * @param input_scale_inv The inverse input scale factor.
 * @param batch_size The batch size.
 * @param input_features The input feature size.
 * @param output_features The output feature size.
 * @param apply_relu Non-zero if ReLU must be applied.
 */
void linear_layer(const int *input, const int8_t *weights, int *output, const int input_scale,
                  const int *weight_scale_inv, const int input_scale_inv,
                  const unsigned int batch_size, const unsigned int input_features,
                  const unsigned int output_features, const unsigned int apply_relu);

/**
 * @brief Runs a quantized 2D convolutional layer with ReLU activation.
 *
 * Input is quantized before convolution and then dequantized per-channel before activation.
 *
 * @param input The (batch_size, input_channels, input_height, input_width) input tensor.
 * @param weights The (output_channels, input_channels, kernel_height, kernel_width) weight tensor.
 * @param output The (batch_size, output_channels, output_height, output_width) output tensor.
 * @param input_scale The scale factor for input quantization.
 * @param weight_scale_inv The inverse scale factors per output channel.
 * @param input_scale_inv The inverse input scale factor.
 * @param batch_size The batch size.
 * @param input_channels The input channels.
 * @param output_channels The output channels.
 * @param input_height The input height.
 * @param input_width The input width.
 * @param output_height The output height.
 * @param output_width The output width.
 * @param kernel_height The kernel height.
 * @param kernel_width The kernel width.
 * @param stride_height The stride height.
 * @param stride_width The stride width.
 */
void conv2d_layer(const int *input, const int8_t *weights, int *output, const int input_scale,
                  const int *weight_scale_inv, const int input_scale_inv,
                  const unsigned int batch_size, const unsigned int input_channels,
                  const unsigned int output_channels, const int input_height, const int input_width,
                  const int output_height, const int output_width, const int kernel_height,
                  const int kernel_width, const int stride_height, const int stride_width);

#endif    // NN_H
```

The implementation of these functions was placed in `src/nn.c`, as shown below:

```c
#include "nn.h"

/**
 * @brief Applies symmetric rounded right-shift on a signed 64-bit integer.
 *
 * Rounding is performed in the magnitude domain and the original sign is restored afterwards,
 * which keeps positive and negative values symmetric.
 *
 * @param value Signed integer value to be shifted.
 * @param shift Number of bits to shift right.
 * @return Rounded and shifted signed value.
 */
static int64_t round_shift_right_symmetric(int64_t value, uint32_t shift)
{
  // Signed symmetric rounded right shift using integer-only arithmetic
  // This avoids bias for negative numbers compared to always adding +0.5 LSB
  if (shift == 0U) {
    return value;
  }

  // Compute absolute value in unsigned domain to avoid INT64_MIN overflow
  const uint64_t abs_value = (value < 0LL) ? (uint64_t)(-(value + 1LL)) + 1ULL : (uint64_t)value;
  const uint64_t rounding_offset = (uint64_t)1U << (shift - 1U);

  // Round in magnitude domain then restore the original sign
  const uint64_t rounded_abs = (abs_value + rounding_offset) >> shift;

  return (value < 0LL) ? -(int64_t)rounded_abs : (int64_t)rounded_abs;
}

/**
 * @brief Clamps a signed 64-bit value to the representable int8 range.
 *
 * The clamp range is symmetric around zero and matches the quantized domain used in this module.
 *
 * @param value Input value in wider integer format.
 * @return Saturated int8 value.
 */
static int8_t saturate_to_int8(int64_t value)
{
  // Saturate signed integer to int8 range used by quantized weights and activations
  if (value > (int64_t)INT8_MAX_VALUE) {
    return (int8_t)INT8_MAX_VALUE;
  }

  if (value < -(int64_t)INT8_MAX_VALUE) {
    return -(int8_t)INT8_MAX_VALUE;
  }

  return (int8_t)value;
}

/**
 * @brief Computes integer matrix multiplication for one batched linear layer.
 *
 * This helper computes: output_matrix = left_matrix x right_matrix where `left_matrix` is
 * [batch_size, input_features] and `right_matrix` is [input_features, output_features], both
 * flattened in row-major order.
 *
 * @param left_matrix Quantized activations matrix.
 * @param right_matrix Quantized weights matrix.
 * @param output_matrix Integer accumulation output matrix.
 * @param batch_size Number of rows in the left matrix.
 * @param input_features Shared dimension between left and right matrices.
 * @param output_features Number of columns in the right matrix.
 */
static void mat_mult(const int8_t *left_matrix, const int8_t *right_matrix, int *output_matrix,
                     const unsigned int batch_size, const unsigned int input_features,
                     const unsigned int output_features)
{
  unsigned int out_feat_idx, batch_idx, in_feat_idx;
  int accumulator;

  for (out_feat_idx = 0; out_feat_idx < output_features; out_feat_idx++) {
    for (batch_idx = 0; batch_idx < batch_size; batch_idx++) {
      // Row-major flattening for one sample row
      const unsigned int in_row_base = batch_idx * input_features;
      accumulator = 0;

      for (in_feat_idx = 0; in_feat_idx < input_features; in_feat_idx++) {
        // Right matrix is stored as [input_features][output_features]
        const unsigned int weight_row_base = in_feat_idx * output_features;
        accumulator +=
          left_matrix[in_row_base + in_feat_idx] * right_matrix[weight_row_base + out_feat_idx];
      }

      output_matrix[batch_idx * output_features + out_feat_idx] = accumulator;
    }
  }
}

/**
 * @brief Performs low-level integer 2D convolution over flattened tensors.
 *
 * All tensors are expected in contiguous row-major layout: input [N, C_in, H_in, W_in], weights
 * [C_out, C_in, K_h, K_w], output [N, C_out, H_out, W_out].
 *
 * @param input Quantized input tensor.
 * @param weights Quantized convolution filters.
 * @param output Output accumulator tensor.
 * @param batch_size Number of input samples.
 * @param input_channels Number of input channels.
 * @param output_channels Number of output channels.
 * @param input_height Input tensor height.
 * @param input_width Input tensor width.
 * @param output_height Output tensor height.
 * @param output_width Output tensor width.
 * @param kernel_height Filter kernel height.
 * @param kernel_width Filter kernel width.
 * @param stride_height Convolution stride in height dimension.
 * @param stride_width Convolution stride in width dimension.
 */
static void conv2d(const int8_t *input, const int8_t *weights, int *output, int batch_size,
                   int input_channels, int output_channels, int input_height, int input_width,
                   int output_height, int output_width, int kernel_height, int kernel_width,
                   int stride_height, int stride_width)
{
  const int input_channel_area = input_height * input_width;
  const int output_channel_area = output_height * output_width;
  const int kernel_size = kernel_height * kernel_width;

  for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    // Base offsets for the current sample in flattened tensors
    const int output_batch_offset = batch_idx * output_channels * output_channel_area;
    const int input_batch_offset = batch_idx * input_channels * input_channel_area;

    for (int out_chan_idx = 0; out_chan_idx < output_channels; out_chan_idx++) {
      // Offsets for the current output channel and its filter bank
      const int output_channel_offset = out_chan_idx * output_channel_area;
      const int kernel_output_channel_offset = out_chan_idx * input_channels * kernel_size;

      for (int output_row = 0; output_row < output_height; output_row++) {
        for (int output_col = 0; output_col < output_width; output_col++) {
          const int output_channel_position = output_row * output_width + output_col;

          // Input origin of the receptive field after stride
          const int input_window_start_offset =
            output_row * stride_height * input_width + output_col * stride_width;

          int sum = 0;

          for (int in_chan_idx = 0; in_chan_idx < input_channels; in_chan_idx++) {
            // Offsets for the current input channel and its kernel slice
            const int input_channel_offset = in_chan_idx * input_channel_area;
            const int kernel_input_channel_offset = in_chan_idx * kernel_size;

            for (int kernel_row = 0; kernel_row < kernel_height; kernel_row++) {
              for (int kernel_col = 0; kernel_col < kernel_width; kernel_col++) {
                const int kernel_position = kernel_row * kernel_width + kernel_col;

                const int input_kernel_offset = kernel_row * input_width + kernel_col;

                const int input_index = input_batch_offset + input_channel_offset +
                                        input_window_start_offset + input_kernel_offset;

                const int weight_index =
                  kernel_output_channel_offset + kernel_input_channel_offset + kernel_position;

                const int input_value = (int)input[input_index];
                const int weight_value = (int)weights[weight_index];

                // Accumulate dot-product in integer domain
                sum += input_value * weight_value;
              }
            }
          }

          const int output_index =
            output_batch_offset + output_channel_offset + output_channel_position;

          output[output_index] = sum;
        }
      }
    }
  }
}

void pooling2d(const int *input, int *output, int batch_size, int channels, int input_height,
               int input_width, int output_height, int output_width, int kernel_height,
               int kernel_width, int stride_height, int stride_width)
{
  const int input_channel_area = input_height * input_width;
  const int output_channel_area = output_height * output_width;

  for (int batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    // Base offsets for the current sample in flattened tensors
    const int output_batch_offset = batch_idx * channels * output_channel_area;
    const int input_batch_offset = batch_idx * channels * input_channel_area;

    for (int channel_index = 0; channel_index < channels; channel_index++) {
      const int output_channel_offset = channel_index * output_channel_area;
      const int input_channel_offset = channel_index * input_channel_area;

      for (int output_row = 0; output_row < output_height; output_row++) {
        for (int output_col = 0; output_col < output_width; output_col++) {
          const int output_channel_position = output_row * output_width + output_col;

          // Top-left corner of the pooling window
          const int input_window_start_offset =
            output_row * stride_height * input_width + output_col * stride_width;

          // Start max with the first value in the current window
          const int first_input_index =
            input_batch_offset + input_channel_offset + input_window_start_offset;

          int max_value = input[first_input_index];

          for (int kernel_row = 0; kernel_row < kernel_height; kernel_row++) {
            for (int kernel_col = 0; kernel_col < kernel_width; kernel_col++) {
              const int input_kernel_offset = kernel_row * input_width + kernel_col;

              const int input_index = input_batch_offset + input_channel_offset +
                                      input_window_start_offset + input_kernel_offset;

              const int value = input[input_index];

              if (value > max_value) {
                max_value = value;
              }
            }
          }

          const int output_index =
            output_batch_offset + output_channel_offset + output_channel_position;

          output[output_index] = max_value;
        }
      }
    }
  }
}

/**
 * @brief Applies ReLU in-place on a flattened integer tensor.
 *
 * @param data Input/output tensor buffer.
 * @param size Number of elements in the flattened tensor.
 */
static void relu(int *data, const unsigned int size)
{
  unsigned int i;

  for (i = 0; i < size; i++) {
    data[i] = MAX(data[i], 0);
  }
}

/**
 * @brief Quantizes Q(FRAC_BITS) activations into int8 values with symmetric rounding.
 *
 * @param input Input tensor in Q(FRAC_BITS) domain.
 * @param output Output tensor in int8 quantized domain.
 * @param input_scale Q(FRAC_BITS) scale factor used for quantization.
 * @param size Number of flattened tensor elements.
 */
static void quantize(const int *input, int8_t *output, const int input_scale,
                     const unsigned int size)
{
  unsigned int i;

  for (i = 0; i < size; i++) {
    // Input and scale are Q(FRAC_BITS) so product is Q(2 * FRAC_BITS)
    const int64_t scaled_product = (int64_t)input[i] * (int64_t)input_scale;

    const int64_t quantized_value =
      round_shift_right_symmetric(scaled_product, (uint32_t)(2 * FRAC_BITS));

    // Saturate before narrowing to int8 to prevent wraparound
    output[i] = saturate_to_int8(quantized_value);
  }
}

/**
 * @brief Dequantizes linear-layer accumulators using per-column scales.
 *
 * After fully-connected matmul, one inverse weight scale is applied to each output column.
 *
 * @param data Matrix to dequantize in-place.
 * @param weight_scale_inv Inverse weight scales, one per column.
 * @param input_scale_inv Inverse input scale.
 * @param batch_size Number of rows in the output matrix.
 * @param output_features Number of columns in the output matrix.
 */
static void dequantize_per_col(int *data, const int *weight_scale_inv, const int input_scale_inv,
                               const unsigned int batch_size, const unsigned int output_features)
{
  unsigned int batch_idx, out_feat_idx;

  for (batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    for (out_feat_idx = 0; out_feat_idx < output_features; out_feat_idx++) {
      // Build the per-column inverse scale and reproject the accumulator back to Q
      const int64_t combined_inv_scale =
        (int64_t)weight_scale_inv[out_feat_idx] * (int64_t)input_scale_inv;

      const unsigned int out_idx = batch_idx * output_features + out_feat_idx;

      const int64_t dequantized_value = round_shift_right_symmetric(
        combined_inv_scale * (int64_t)data[out_idx], (uint32_t)FRAC_BITS);

      data[out_idx] = (int)dequantized_value;
    }
  }
}

/**
 * @brief Dequantizes convolution accumulators using per-output-channel scales.
 *
 * @param data Tensor to dequantize in-place, flattened as [N, C_out, ...].
 * @param weight_scale_inv Inverse weight scales, one per output channel.
 * @param input_scale_inv Inverse input activation scale.
 * @param batch_size Number of input samples.
 * @param output_channels Number of output channels.
 * @param output_size Flattened output spatial size per channel.
 */
static void dequantize_per_channel(int *data, const int *weight_scale_inv,
                                   const int input_scale_inv, const unsigned int batch_size,
                                   const unsigned int output_channels,
                                   const unsigned int output_size)
{
  unsigned int batch_idx, out_chan_idx, out_pos_idx;

  for (batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    for (out_chan_idx = 0; out_chan_idx < output_channels; out_chan_idx++) {
      // This scale is constant across all features of the same output channel
      const int64_t combined_inv_scale =
        (int64_t)weight_scale_inv[out_chan_idx] * (int64_t)input_scale_inv;

      for (out_pos_idx = 0; out_pos_idx < output_size; out_pos_idx++) {
        // Flatten (n, c, k) into one index for the contiguous tensor buffer
        const unsigned int out_idx =
          batch_idx * output_channels * output_size + out_chan_idx * output_size + out_pos_idx;

        const int64_t scaled_accumulator = combined_inv_scale * (int64_t)data[out_idx];

        const int64_t dequantized_value =
          round_shift_right_symmetric(scaled_accumulator, (uint32_t)FRAC_BITS);

        data[out_idx] = (int)dequantized_value;
      }
    }
  }
}

void argmax_per_row(const int *input, unsigned int *indices, const unsigned int batch_size,
                    const unsigned int output_features)
{
  unsigned int batch_idx, out_feat_idx, max_out_feat_idx;
  int row_max;

  for (batch_idx = 0; batch_idx < batch_size; batch_idx++) {
    // Initialize with the first element then scan the whole row
    const unsigned int row_base = batch_idx * output_features;
    row_max = input[row_base];
    max_out_feat_idx = 0;

    for (out_feat_idx = 0; out_feat_idx < output_features; out_feat_idx++) {
      const int value = input[row_base + out_feat_idx];

      if (value > row_max) {
        row_max = value;
        max_out_feat_idx = out_feat_idx;
      }
    }

    indices[batch_idx] = max_out_feat_idx;
  }
}

void linear_layer(const int *input, const int8_t *weights, int *output, const int input_scale,
                  const int *weight_scale_inv, const int input_scale_inv,
                  const unsigned int batch_size, const unsigned int input_features,
                  const unsigned int output_features, const unsigned int apply_relu)
{
  int8_t input_quantized[batch_size * input_features];

  // Quantize Q(FRAC_BITS) activations to int8 domain expected by integer GEMM
  quantize(input, input_quantized, input_scale, batch_size * input_features);

  // Integer matmul gives accumulators in int domain
  mat_mult(input_quantized, weights, output, batch_size, input_features, output_features);

  // Bring accumulators back to Q(FRAC_BITS) for the next layer
  dequantize_per_col(output, weight_scale_inv, input_scale_inv, batch_size, output_features);

  if (apply_relu) {
    relu(output, batch_size * output_features);
  }
}

void conv2d_layer(const int *input, const int8_t *weights, int *output, const int input_scale,
                  const int *weight_scale_inv, const int input_scale_inv,
                  const unsigned int batch_size, const unsigned int input_channels,
                  const unsigned int output_channels, const int input_height, const int input_width,
                  const int output_height, const int output_width, const int kernel_height,
                  const int kernel_width, const int stride_height, const int stride_width)
{
  int8_t input_quantized[batch_size * input_channels * input_height * input_width];

  // Quantize Q(FRAC_BITS) activations to int8 domain expected by convolution
  quantize(input, input_quantized, input_scale,
           batch_size * input_channels * input_height * input_width);

  // Integer convolution on quantized activations and weights
  conv2d(input_quantized, weights, output, batch_size, input_channels, output_channels,
         input_height, input_width, output_height, output_width, kernel_height, kernel_width,
         stride_height, stride_width);

  // Bring accumulators back to Q(FRAC_BITS) then apply non-linearity
  dequantize_per_channel(output, weight_scale_inv, input_scale_inv, batch_size, output_channels,
                         output_height * output_width);

  relu(output, batch_size * output_channels * output_height * output_width);
}
```

The complete network was implemented in `src/convnet.c`. The `convnet_forward` function connected the layers in sequence using the dimensions and model parameters declared in `params.h`. It ran two stages of convolution followed by max-pooling, then three linear layers. ReLU was applied after both convolutions and the first two linear layers, while the last layer kept its signed output values. Finally, `argmax_per_row` selected the index of the largest of the ten output values to obtain the predicted digit. The code calling `convnet_forward` provided arrays to store the output of each stage, making the intermediate results available for inspection during evaluation.

The function declarations were placed in `include/convnet.h`, as shown below:

```c
#ifndef CONVNET_H
#define CONVNET_H

#define BATCH_SIZE 1

#include "params.h"

/**
 * @brief Runs the pre-defined convolutional neural network.
 *
 * @param input The input tensor.
 * @param predictions The [batch_size x 1] vector storing predicted class indices.
 */
void convnet_run(const int *input, unsigned int *predictions);

/**
 * @brief Shared neural-network forward function with intermediate outputs.
 *
 * @param input The flattened fixed-point input tensor.
 * @param conv_1_output The first convolution output buffer.
 * @param pool_1_output The first max-pooling output buffer.
 * @param conv_2_output The second convolution output buffer.
 * @param pool_2_output The second max-pooling output buffer.
 * @param linear_1_output The first linear layer output buffer.
 * @param linear_2_output The second linear layer output buffer.
 * @param logits The final logits buffer.
 * @param predictions The [batch_size x 1] vector storing predicted class indices.
 */
void convnet_forward(const int *input, int *conv_1_output, int *pool_1_output, int *conv_2_output,
                     int *pool_2_output, int *linear_1_output, int *linear_2_output, int *logits,
                     unsigned int *predictions);

#endif    // CONVNET_H
```

The implementation of these functions was placed in `src/convnet.c`, as shown below:

```c
#include "convnet.h"
#include "nn.h"

void convnet_forward(const int *input, int *conv_1_output, int *pool_1_output, int *conv_2_output,
                     int *pool_2_output, int *linear_1_output, int *linear_2_output, int *logits,
                     unsigned int *predictions)
{
    // conv1: input -> conv -> dequant -> ReLU
    conv2d_layer(input, conv_1_weight, conv_1_output, conv_1_input_scale, conv_1_weight_scale_inv,
                 conv_1_input_scale_inv, BATCH_SIZE, INPUT_CHANNELS, CONV_1_OUT_CHANNELS,
                 INPUT_HEIGHT, INPUT_WIDTH, CONV_1_OUT_HEIGHT, CONV_1_OUT_WIDTH, 5, 5, 1, 1);

    // pool1: downsample conv1 features
    pooling2d(conv_1_output, pool_1_output, BATCH_SIZE, CONV_1_OUT_CHANNELS, CONV_1_OUT_HEIGHT,
              CONV_1_OUT_WIDTH, POOL_1_OUT_HEIGHT, POOL_1_OUT_WIDTH, 2, 2, 2, 2);

    // conv2: pool1 -> conv -> dequant -> ReLU
    conv2d_layer(pool_1_output, conv_2_weight, conv_2_output, conv_2_input_scale,
                 conv_2_weight_scale_inv, conv_2_input_scale_inv, BATCH_SIZE, CONV_1_OUT_CHANNELS,
                 CONV_2_OUT_CHANNELS, POOL_1_OUT_HEIGHT, POOL_1_OUT_WIDTH, CONV_2_OUT_HEIGHT,
                 CONV_2_OUT_WIDTH, 5, 5, 1, 1);

    // pool2: downsample conv2 features
    pooling2d(conv_2_output, pool_2_output, BATCH_SIZE, CONV_2_OUT_CHANNELS, CONV_2_OUT_HEIGHT,
              CONV_2_OUT_WIDTH, POOL_2_OUT_HEIGHT, POOL_2_OUT_WIDTH, 2, 2, 2, 2);

    // linear stack: pool_2 -> linear_1 -> linear_2 -> linear_3
    linear_layer(pool_2_output, linear_1_weight, linear_1_output, linear_1_input_scale,
                 linear_1_weight_scale_inv, linear_1_input_scale_inv, BATCH_SIZE,
                 CONV_2_OUT_CHANNELS * POOL_2_OUT_HEIGHT * POOL_2_OUT_WIDTH, LINEAR_1_OUT_FEATURES,
                 1);

    linear_layer(linear_1_output, linear_2_weight, linear_2_output, linear_2_input_scale,
                 linear_2_weight_scale_inv, linear_2_input_scale_inv, BATCH_SIZE,
                 LINEAR_1_OUT_FEATURES, LINEAR_2_OUT_FEATURES, 1);

    linear_layer(linear_2_output, linear_3_weight, logits, linear_3_input_scale,
                 linear_3_weight_scale_inv, linear_3_input_scale_inv, BATCH_SIZE,
                 LINEAR_2_OUT_FEATURES, OUTPUT_DIM, 0);

    // final class index from logits
    argmax_per_row(logits, predictions, BATCH_SIZE, OUTPUT_DIM);
}

void convnet_run(const int *input, unsigned int *predictions)
{
    int conv_1_output[BATCH_SIZE * CONV_1_OUT_CHANNELS * CONV_1_OUT_HEIGHT * CONV_1_OUT_WIDTH];
    int pool_1_output[BATCH_SIZE * CONV_1_OUT_CHANNELS * POOL_1_OUT_HEIGHT * POOL_1_OUT_WIDTH];
    int conv_2_output[BATCH_SIZE * CONV_2_OUT_CHANNELS * CONV_2_OUT_HEIGHT * CONV_2_OUT_WIDTH];
    int pool_2_output[BATCH_SIZE * CONV_2_OUT_CHANNELS * POOL_2_OUT_HEIGHT * POOL_2_OUT_WIDTH];
    int linear_1_output[BATCH_SIZE * LINEAR_1_OUT_FEATURES];
    int linear_2_output[BATCH_SIZE * LINEAR_2_OUT_FEATURES];
    int logits[BATCH_SIZE * OUTPUT_DIM];

    convnet_forward(input, conv_1_output, pool_1_output, conv_2_output, pool_2_output,
                    linear_1_output, linear_2_output, logits, predictions);
}
```

The three C sources were compiled into a shared library with the commands shown below, which allowed the engine to be called from Python for evaluation:

```bash
mkdir -p lib

gcc -O3 -std=c11 -fPIC -shared -Iinclude src/convnet.c src/nn.c src/params.c -o lib/convnet.so
```

The next section describes how the quantized model implemented in this library was compared against the floating-point model.

### Model Evaluation

To evaluate the effect of quantization, a script named `eval.py` was created inside the `scripts` directory. It compared the predictions of the integer-only C engine with those of the original floating-point model on the MNIST test set.

First, the script loaded `lib/convnet.so` through `ctypes`, allowing the C inference code to be called directly from Python. The original PyTorch model was then restored from `models/model.pt` and set to evaluation mode. The 10,000 test images were converted to tensors and normalized in the same way as during training, but without data augmentation, so both models received the same image data. For each image, the PyTorch model produced its prediction from the largest output value. The same normalized image was then converted to Q16 by multiplying its values by $$2^{16}$$ and rounding them to integers. This matched the input format expected by the C engine, which returned its own predicted digit.

During this comparison, the script counted the correct predictions from each model and how often their predictions agreed. It also counted cases where only one model was correct. This showed when quantization changed a correct prediction into an error and when it corrected an error made by the floating-point model.

Finally, the saved predictions and true labels were used to calculate accuracy, precision, recall, and F1-score for both models. The script printed these metrics and the prediction agreement, then saved the two confusion matrices side by side in `images/confusion_matrices.png` to show which digits were confused. The complete script is shown below:

```python
import argparse
import ctypes
from pathlib import Path

import matplotlib
import numpy as np
import torch
import torchvision.datasets as datasets
import torchvision.transforms as transforms
from model import ConvNet
from torch.utils.data import DataLoader

# Use a non-interactive backend so plots can be generated in headless environments
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_c_lib(library_path: Path):
    """
    @brief Loads the compiled C shared library.

    @param library_path Path to the compiled shared object.
    @return ctypes library handle.
    """
    try:
        return ctypes.CDLL(str(library_path.resolve()))
    except OSError as exc:
        raise RuntimeError(f"Unable to load C library: {library_path}") from exc


def ensure_contiguous(array: np.ndarray) -> np.ndarray:
    """
    @brief Ensures an array is C-contiguous.

    @param array Input NumPy array.
    @return C-contiguous array.
    """
    return np.ascontiguousarray(array) if not array.flags["C_CONTIGUOUS"] else array


def convnet_run(sample: torch.Tensor, c_lib) -> int:
    """
    @brief Runs one quantized C inference sample.

    @param sample Input sample tensor in fixed-point integer format.
    @param c_lib Loaded C shared library.
    @return Predicted class index.
    """
    # Flatten image tensor to match the C API expected 1D input layout
    x = sample.flatten().cpu().numpy().astype(np.intc)
    x = ensure_contiguous(x)

    # Allocate output buffer for the predicted class index
    pred = ensure_contiguous(np.zeros(1, dtype=np.uintc))

    # Mirror C pointer types used by convnet_run(const int*, unsigned int*)
    c_int_p = ctypes.POINTER(ctypes.c_int)
    c_uint_p = ctypes.POINTER(ctypes.c_uint)

    c_convnet_run = c_lib.convnet_run
    c_convnet_run(x.ctypes.data_as(c_int_p), pred.ctypes.data_as(c_uint_p))

    return int(pred[0])


def load_float_model(model_path: Path) -> ConvNet:
    """
    @brief Loads the floating-point model checkpoint.

    @param model_path Path to float model checkpoint.
    @return Loaded ConvNet model in eval mode.
    """
    if not model_path.exists():
        raise FileNotFoundError(f"Model checkpoint not found: {model_path}")

    saved_stats = torch.load(model_path, map_location="cpu", weights_only=False)

    if isinstance(saved_stats, dict) and "state_dict" in saved_stats:
        state_dict = saved_stats["state_dict"]
    else:
        state_dict = saved_stats

    model = ConvNet(h=28, w=28, inputs=1, outputs=10)
    model.load_state_dict(state_dict)
    model.eval()
    return model


def build_confusion_matrix(labels: np.ndarray, predictions: np.ndarray, num_classes: int):
    """
    @brief Builds a confusion matrix from labels and predictions.

    @param labels Ground-truth labels.
    @param predictions Predicted labels.
    @param num_classes Number of classes.
    @return Confusion matrix of shape num_classes x num_classes.
    """
    confusion_matrix = np.zeros((num_classes, num_classes), dtype=np.int64)
    np.add.at(confusion_matrix, (labels, predictions), 1)

    return confusion_matrix


def compute_classification_metrics(confusion_matrix: np.ndarray):
    """
    @brief Computes common classification metrics from a confusion matrix.

    @param confusion_matrix Confusion matrix with rows=true and cols=predicted.
    @return Dictionary with accuracy, precision, recall and f1 averages.
    """
    # Rows are true labels and columns are predicted labels
    true_positives = np.diag(confusion_matrix).astype(np.float64)
    support = confusion_matrix.sum(axis=1).astype(np.float64)
    predicted_totals = confusion_matrix.sum(axis=0).astype(np.float64)

    false_positives = predicted_totals - true_positives
    false_negatives = support - true_positives

    # Use guarded division to avoid NaNs for classes with zero support/predictions
    precision = np.divide(true_positives,
                          true_positives + false_positives,
                          out=np.zeros_like(true_positives),
                          where=(true_positives + false_positives) != 0)
    
    recall = np.divide(true_positives,
                       true_positives + false_negatives,
                       out=np.zeros_like(true_positives),
                       where=(true_positives + false_negatives) != 0)
    
    f1_score = np.divide(2.0 * precision * recall,
                         precision + recall,
                         out=np.zeros_like(true_positives),
                         where=(precision + recall) != 0)

    total_samples = float(confusion_matrix.sum())
    accuracy = float(true_positives.sum() / total_samples) if total_samples > 0 else 0.0

    precision_macro = float(np.mean(precision))
    recall_macro = float(np.mean(recall))
    f1_macro = float(np.mean(f1_score))

    # Weighted metrics account for class imbalance via per-class support
    weighted_precision = (
        float(np.sum(precision * support) / total_samples) if total_samples > 0 else 0.0
    )

    weighted_recall = (
        float(np.sum(recall * support) / total_samples) if total_samples > 0 else 0.0
    )
    
    weighted_f1 = (
        float(np.sum(f1_score * support) / total_samples) if total_samples > 0 else 0.0
    )

    return {
        "accuracy": accuracy,
        "precision_macro": precision_macro,
        "recall_macro": recall_macro,
        "f1_macro": f1_macro,
        "precision_weighted": weighted_precision,
        "recall_weighted": weighted_recall,
        "f1_weighted": weighted_f1,
    }


def draw_confusion_matrix(ax, confusion_matrix: np.ndarray, title: str):
    """
    @brief Draws one confusion matrix on a given axis.

    @param ax Matplotlib axis where the matrix is drawn.
    @param confusion_matrix Confusion matrix to plot.
    @param title Plot title.
    """
    image = ax.imshow(confusion_matrix, cmap="Blues")

    num_classes = confusion_matrix.shape[0]
    ticks = np.arange(num_classes)
    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.set_xlabel("Predicted label")
    ax.set_ylabel("True label")
    ax.set_title(title)

    # Dynamic text color keeps counts readable over light and dark cells
    max_value = confusion_matrix.max() if confusion_matrix.size > 0 else 0
    threshold = max_value * 0.5

    for row_idx in range(num_classes):
        for col_idx in range(num_classes):
            color = "white" if confusion_matrix[row_idx, col_idx] > threshold else "black"
            ax.text(col_idx, row_idx, str(confusion_matrix[row_idx, col_idx]),
                    ha="center", va="center", color=color, fontsize=8)

    return image


def plot_confusion_matrices(original_confusion: np.ndarray, quantized_confusion: np.ndarray,
                            output_path: Path):
    """
    @brief Plots and saves both confusion matrices in a single image.

    @param original_confusion Confusion matrix of original model.
    @param quantized_confusion Confusion matrix of quantized model.
    @param output_path Output image path.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    original_image = draw_confusion_matrix(axes[0],
                                           original_confusion,
                                           "Original Model Confusion Matrix")
    
    quantized_image = draw_confusion_matrix(axes[1],
                                            quantized_confusion,
                                            "Quantized Model Confusion Matrix")

    fig.colorbar(original_image, ax=axes[0], fraction=0.046, pad=0.04)
    fig.colorbar(quantized_image, ax=axes[1], fraction=0.046, pad=0.04)
    fig.tight_layout()
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_evaluation_report(total: int, correct_original: int,
                            original_accuracy: float, correct_quantized: int,
                            quantized_accuracy: float, same_prediction: int,
                            agreement: float, quantized_only_correct: int,
                            original_only_correct: int, original_metrics: dict,
                            quantized_metrics: dict,
                            confusion_matrices_path: Path):
    """
    @brief Prints the complete evaluation report in one place.

    @param total Number of evaluated samples.
    @param correct_original Number of correct predictions from original model.
    @param original_accuracy Original model accuracy in percentage.
    @param correct_quantized Number of correct predictions from quantized model.
    @param quantized_accuracy Quantized model accuracy in percentage.
    @param same_prediction Number of equal predictions between both models.
    @param agreement Prediction agreement in percentage.
    @param quantized_only_correct Correct only by quantized model.
    @param original_only_correct Correct only by original model.
    @param original_metrics Metrics dictionary for original model.
    @param quantized_metrics Metrics dictionary for quantized model.
    @param confusion_matrices_path Path to combined confusion matrices image.
    """
    print(f"Samples: {total}")
    print("\n")
    print("--- Accuracy Report ---")
    print(f"Original model correct: {correct_original}")
    print(f"Original model accuracy: {original_accuracy:.2f}%")
    print(f"Quantized model correct: {correct_quantized}")
    print(f"Quantized model accuracy: {quantized_accuracy:.2f}%")
    print("\n")
    print("--- Comparison ---")
    print(f"Prediction agreement: {same_prediction} ({agreement:.2f}%)")
    print(f"Quantized only correct: {quantized_only_correct}")
    print(f"Original only correct: {original_only_correct}")
    print("\n")

    metric_rows = [
        ("Accuracy", "accuracy"),
        ("Precision (macro)", "precision_macro"),
        ("Recall (macro)", "recall_macro"),
        ("F1-score (macro)", "f1_macro"),
        ("Precision (weighted)", "precision_weighted"),
        ("Recall (weighted)", "recall_weighted"),
        ("F1-score (weighted)", "f1_weighted"),
    ]

    print("--- Classification Metrics Matrix (%) ---")
    print(f"{'Metric':<24}{'Original':>12}{'Quantized':>14}")
    print("-" * 50)

    for metric_name, metric_key in metric_rows:
        original_value = original_metrics[metric_key] * 100.0
        quantized_value = quantized_metrics[metric_key] * 100.0
        print(f"{metric_name:<24}{original_value:>12.2f}{quantized_value:>14.2f}")

    print("\n")
    print(f"Confusion matrices saved: {confusion_matrices_path}")


def main(args):
    c_lib = load_c_lib(args.lib_path)
    original_model = load_float_model(args.float_model)

    # Declare C function signature for safe ctypes calls
    c_int_p = ctypes.POINTER(ctypes.c_int)
    c_uint_p = ctypes.POINTER(ctypes.c_uint)
    c_lib.convnet_run.argtypes = (c_int_p, c_uint_p)
    c_lib.convnet_run.restype = None

    dataset = datasets.MNIST(root=str(args.data_dir),
                             train=False,
                             download=True,
                             transform=transforms.Compose([transforms.ToTensor(),
                                                           transforms.Normalize((0.5,), (0.5,))]))
    data_loader = DataLoader(dataset,
                             batch_size=args.batch_size,
                             shuffle=False,
                             num_workers=args.num_workers)

    # Ensure output directory exists before saving figures
    args.images_dir.mkdir(parents=True, exist_ok=True)

    # Track accuracy, agreement, and disagreement buckets between both models
    total = 0
    correct_quantized = 0
    correct_original = 0
    same_prediction = 0
    quantized_only_correct = 0
    original_only_correct = 0

    # Save all predictions to build confusion matrices at the end
    all_labels = []
    all_original_predictions = []
    all_quantized_predictions = []

    with torch.no_grad():
        for samples, labels in data_loader:
            original_logits = original_model(samples)
            original_predictions = torch.argmax(original_logits, dim=1)

            # Convert normalized float input to fixed-point integer format expected by C
            samples_q = (samples * (1 << args.frac_bits)).round().to(torch.int32)

            batch_size = labels.shape[0]
            for idx in range(batch_size):
                quantized_prediction = convnet_run(samples_q[idx], c_lib)
                original_prediction = int(original_predictions[idx])
                label = int(labels[idx])

                is_quantized_correct = int(quantized_prediction == label)
                is_original_correct = int(original_prediction == label)

                correct_quantized += is_quantized_correct
                correct_original += is_original_correct
                same_prediction += int(quantized_prediction == original_prediction)
                quantized_only_correct += int(is_quantized_correct and not is_original_correct)
                original_only_correct += int(is_original_correct and not is_quantized_correct)
                total += 1

                all_labels.append(label)
                all_original_predictions.append(original_prediction)
                all_quantized_predictions.append(quantized_prediction)

                # Stop early when max_samples is set for quick checks
                if args.max_samples > 0 and total >= args.max_samples:
                    break

            if args.max_samples > 0 and total >= args.max_samples:
                break

    # Convert lists to arrays for vectorized confusion matrix and metrics computation
    labels_array = np.asarray(all_labels, dtype=np.int64)
    original_predictions_array = np.asarray(all_original_predictions, dtype=np.int64)
    quantized_predictions_array = np.asarray(all_quantized_predictions, dtype=np.int64)

    num_classes = 10

    original_confusion = build_confusion_matrix(labels_array,
                                                original_predictions_array,
                                                num_classes)
    
    quantized_confusion = build_confusion_matrix(labels_array,
                                                 quantized_predictions_array,
                                                 num_classes)

    original_metrics = compute_classification_metrics(original_confusion)
    quantized_metrics = compute_classification_metrics(quantized_confusion)

    quantized_accuracy = 100.0 * correct_quantized / total if total > 0 else 0.0
    original_accuracy = 100.0 * correct_original / total if total > 0 else 0.0
    agreement = 100.0 * same_prediction / total if total > 0 else 0.0

    confusion_matrices_path = args.images_dir / "confusion_matrices.png"

    # Save both confusion matrices into a single side-by-side image
    plot_confusion_matrices(original_confusion,
                            quantized_confusion,
                            confusion_matrices_path)

    print_evaluation_report(total, correct_original, original_accuracy, correct_quantized,
                            quantized_accuracy, same_prediction, agreement,
                            quantized_only_correct, original_only_correct,
                            original_metrics, quantized_metrics,
                            confusion_matrices_path)


if __name__ == "__main__":
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent

    parser = argparse.ArgumentParser(description="Run quantized C ConvNet inference "
                                                 "on MNIST test data.",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    
    parser.add_argument("--lib_path", type=Path, default=project_root / "lib" / "convnet.so",
                        help="Path to compiled C shared library.")
    
    parser.add_argument("--float_model", type=Path, default=project_root / "models" / "model.pt",
                        help="Path to floating-point model checkpoint used by Python inference.")
    
    parser.add_argument("--data_dir", type=Path, default=project_root / "data",
                        help="Directory used by torchvision MNIST dataset.")
    
    parser.add_argument("--images_dir", type=Path, default=project_root / "images",
                        help="Directory where confusion matrix plots are saved.")
    
    parser.add_argument("--batch_size", type=int, default=1, help="Batch size.")

    parser.add_argument("--num_workers", type=int, default=1, help="DataLoader workers.")

    parser.add_argument("--frac_bits", type=int, default=16,
                        help="Fixed-point fractional bits used across C inference.")
    
    parser.add_argument("--max_samples", type=int, default=0,
                        help="If > 0, evaluate only this many samples.")

    main(parser.parse_args())
```

The evaluation was executed from the project root with the command below, using the same number of fractional bits as the generated parameters:

```bash
python3 scripts/eval.py --frac_bits 16
```

The evaluation metrics and confusion matrices produced by this script are discussed in the [Results](#results) section. With the predictions checked against the original model, the quantization pipeline was complete: the floating-point network trained in PyTorch had been converted to `int8` weights and Q16 scale constants and executed using integer-only C code. The next step was to test the same engine on a microcontroller.

### Embedded Deployment

To test the engine on embedded hardware, the same C source files were compiled into firmware for an [STM32 NUCLEO-F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) development board. This board uses an STM32F446RE microcontroller with a 32-bit Arm Cortex-M4 core, 512 KiB of flash memory, and 128 KiB of SRAM. For this deployment, the microcontroller was clocked at 84 MHz, and the firmware was developed with STM32CubeIDE and the HAL library. Although the core includes a floating-point unit, the entire forward pass used integer instructions, allowing the same inference code to be used on cores without an FPU. The complete firmware project, including the inference engine as a submodule, is available on [GitHub](https://github.com/workabotic/stm32_digit_recognition).

An [OV2640](https://www.uctronics.com/download/cam_module/OV2640DS.pdf) camera module was connected to the board through the DCMI peripheral, with DMA transferring each frame into RAM and the SCCB interface used to configure the sensor for $$160 \times 120$$ YUV422 capture. In a continuous loop, the firmware converted the frame to grayscale, center-cropped it to $$120 \times 120$$, and resized it to the $$28 \times 28$$ input expected by the network. The normalization applied during training was replaced by a lookup table that mapped each 8-bit pixel directly to its Q16 value, so no floating-point operation was needed even in the preprocessing. The `convnet_forward` function was then called with the prepared input, and the predicted digit was obtained from the outputs of the model.

The predicted digit was shown on a common-cathode seven-segment display driven by seven GPIO pins, and the firmware also logged predictions and inference times over UART, allowing captured frames to be transferred to a computer for debugging. Figure 7 shows the prototype recognizing a handwritten digit in real time.

![Prototype running digit recognition on the STM32]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/prototype.gif)
*Figure 7 — Prototype running digit recognition on the STM32 NUCLEO-F446RE.*

## Results

The evaluation results showed that the quantized network closely reproduced the behavior of the original model. Using `scripts/eval.py`, both models were tested on all 10,000 handwritten digits in the MNIST test set. The floating-point model classified 9,673 samples correctly (96.73%), while the integer-only model classified 9,672 correctly (96.72%). In addition to achieving similar accuracy, the two models agreed on 9,964 predictions, or 99.64% of the test set. Among the disagreements, 17 samples were classified correctly only by the quantized model and 18 only by the original model. This balance suggests that quantization shifted a small number of borderline predictions in both directions, with little effect on overall accuracy. The table below summarizes the classification metrics: precision remained unchanged, while recall and F1-score each decreased by 0.01 percentage point.

| Metric | Original (`float32`) | Quantized (`int8`) |
|:---|:---:|:---:|
| Accuracy (%) | 96.73 | 96.72 |
| Precision (%) | 96.73 | 96.73 |
| Recall (%) | 96.76 | 96.75 |
| F1-score (%) | 96.74 | 96.73 |
| Parameter storage (KiB) | 172.62 | 44.12 |

*Table 1 — Comparison between the original and the quantized model on the MNIST test set.*

The confusion matrices of both models, generated by the evaluation script and shown in the image below, were nearly identical. The largest source of error was the same in both cases, with the digit 1 being confused with 7 in 46 samples by the original model and in 45 by the quantized one, and every other cell differed by at most three samples.

![Confusion matrices of the original and quantized models]({{ site.url }}{{ site.baseurl }}/public/images/integer-only-neural-network-inference/confusion_matrices.webp)
*Figure 8 — Confusion matrices of the original and quantized models on the MNIST test set.*

Quantization also reduced the memory needed to store the model parameters. The network's 44,190 weights occupied 176,760 bytes (172.62 KiB) when stored as `float32`. After quantization, the same weights required 44,190 bytes as `int8`. The 246 Q16 scale constants, stored as `int32`, added another 984 bytes, bringing the total to 45,174 bytes (44.12 KiB). This reduced parameter storage by almost a factor of four.

Finally, on the STM32 NUCLEO-F446RE, the execution time of the network was averaged over 100 inferences, considering only the neural network execution time and excluding camera capture, grayscale conversion, cropping and resizing, UART logging, and display updates. The average inference time was 335.71 ms, which was enough for the prototype to recognize handwritten digits in real time, as shown in Figure 7.

## Conclusion

The proof of concept for integer-only neural network inference was successfully completed. A LeNet-based CNN trained in PyTorch was quantized to `int8` weights and Q16 scale constants with post-training quantization, converted into C source files, and executed by an inference engine that relied on integer arithmetic only. On the MNIST test set, the quantized model reached 96.72% accuracy, only 0.01 percentage point below the floating-point model, and agreed with it on 99.64% of the predictions, while the parameter storage fell from 172.62 KiB to 44.12 KiB.

Some final considerations relate to the simplifications adopted in this project. The network was trained without bias terms and used only ReLU activations to keep the integer pipeline simple, so supporting other architectures would require accumulator-domain bias quantization and integer approximations of other activation functions. Post-training quantization was sufficient for this task, but the calibration method has a direct impact on accuracy, and quantization-aware training could recover the remaining difference for models that are more sensitive to quantization noise.

Regarding embedded performance, the STM32F446RE completed one inference in 335.71 ms on average using integer instructions only. Using integer arithmetic also makes the approach suitable for microcontrollers without floating-point hardware. The engine was implemented with straightforward nested loops to keep the calculations easy to follow. To investigate possible reductions in latency, further work should first identify the most time-consuming operations and then test optimizations suited to the target processor, such as specialized multiply-accumulate instructions or optimized libraries.
