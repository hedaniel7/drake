#include <cstring>

#include "drake/bindings/pydrake/common/deprecation_pybind.h"
#include "drake/bindings/pydrake/common/serialize_pybind.h"
#include "drake/bindings/pydrake/documentation_pybind.h"
#include "drake/bindings/pydrake/pydrake_pybind.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_interface.h"

namespace drake {
namespace pydrake {

PYBIND11_MODULE(lcm, m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::lcm;
  constexpr auto& doc = pydrake_doc.drake.lcm;

  py::module::import("pydrake.common");

  // Use `py::bytes` as a mid-point between C++ LCM (`void* + int` /
  // `vector<uint8_t>`) and Python LCM (`str`).
  using PyHandlerFunction = std::function<void(py::bytes)>;
  using PyMultichannelHandlerFunction =
      std::function<void(std::string_view, py::bytes)>;

  {
    using Class = DrakeLcmInterface;
    constexpr auto& cls_doc = doc.DrakeLcmInterface;
    py::class_<Class>(m, "DrakeLcmInterface", cls_doc.doc)
        .def("get_lcm_url", &DrakeLcmInterface::get_lcm_url,
            cls_doc.get_lcm_url.doc)
        .def(
            "Publish",
            [](Class* self, const std::string& channel, py::bytes buffer,
                std::optional<double> time_sec) {
              // TODO(eric.cousineau): See if there is a way to extra the raw
              // bytes from `buffer` without copying.
              std::string str = buffer;
              self->Publish(channel, str.data(), str.size(), time_sec);
            },
            py::arg("channel"), py::arg("buffer"),
            py::arg("time_sec") = py::none(), cls_doc.Publish.doc)
        .def(
            "Subscribe",
            [](Class* self, const std::string& channel,
                PyHandlerFunction handler) {
              auto subscription = self->Subscribe(
                  channel, [handler](const void* data, int size) {
                    handler(py::bytes(static_cast<const char*>(data), size));
                  });
              DRAKE_DEMAND(subscription != nullptr);
              // This is already the default, but for clarity we'll repeat it.
              subscription->set_unsubscribe_on_delete(false);
            },
            py::arg("channel"), py::arg("handler"), cls_doc.Subscribe.doc)
        .def(
            "SubscribeMultichannel",
            [](Class* self, const std::string& regex,
                PyMultichannelHandlerFunction handler) {
              auto subscription = self->SubscribeMultichannel(
                  regex, [handler](std::string_view channel, const void* data,
                             int size) {
                    handler(channel,
                        py::bytes(static_cast<const char*>(data), size));
                  });
              DRAKE_DEMAND(subscription != nullptr);
              // This is already the default, but for clarity we'll repeat it.
              subscription->set_unsubscribe_on_delete(false);
            },
            py::arg("regex"), py::arg("handler"),
            cls_doc.SubscribeMultichannel.doc)
        .def(
            "SubscribeAllChannels",
            [](Class* self, PyMultichannelHandlerFunction handler) {
              auto subscription =
                  self->SubscribeAllChannels([handler](std::string_view channel,
                                                 const void* data, int size) {
                    handler(channel,
                        py::bytes(static_cast<const char*>(data), size));
                  });
              DRAKE_DEMAND(subscription != nullptr);
              // This is already the default, but for clarity we'll repeat it.
              subscription->set_unsubscribe_on_delete(false);
            },
            py::arg("handler"), cls_doc.SubscribeAllChannels.doc)
        .def("HandleSubscriptions", &DrakeLcmInterface::HandleSubscriptions,
            py::arg("timeout_millis"), cls_doc.HandleSubscriptions.doc);
  }

  {
    using Class = DrakeLcmParams;
    constexpr auto& cls_doc = doc.DrakeLcmParams;
    py::class_<Class> cls(m, "DrakeLcmParams", cls_doc.doc);
    cls  // BR
        .def(ParamInit<Class>());
    DefAttributesUsingSerialize(&cls, cls_doc);
    DefReprUsingSerialize(&cls);
    DefCopyAndDeepCopy(&cls);
  }

  {
    using Class = DrakeLcm;
    constexpr auto& cls_doc = doc.DrakeLcm;
    py::class_<Class, DrakeLcmInterface> cls(m, "DrakeLcm", cls_doc.doc);
    cls  // BR
        .def(py::init<>(), cls_doc.ctor.doc_0args)
        .def(py::init<std::string>(), py::arg("lcm_url"),
            cls_doc.ctor.doc_1args_lcm_url)
        .def(py::init<DrakeLcmParams>(), py::arg("params"),
            cls_doc.ctor.doc_1args_params);
  }

  ExecuteExtraPythonCode(m);
}

}  // namespace pydrake
}  // namespace drake
