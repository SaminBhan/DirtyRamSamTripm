#!/usr/bin/env python3 
import numpy as np 


if __name__=="__main__":

    #!/usr/bin/env python3
    print("Hello world")

    np.random.seed(42)

    A = np.random.normal(size=(4, 4))
    B = np.random.normal(size=(4, 2))

    print(A @ B)
    np.random.seed(42)
    x = np.random.normal(size=(4, 10))

    x1 = x[None, :, :]
    x2 = x[:, None, :]

    out = x1 - x2 

    print(out.shape)

    out = np.square(out)
    print(out.shape)

    out = np.sum(out, axis=-1)

    print(out.shape)

    print(out)
