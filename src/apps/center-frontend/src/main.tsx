import ReactDOM from "react-dom/client";
import { QueryClient, QueryClientProvider } from "@tanstack/react-query";
import { App } from "./App";
import { userStorage } from "./api/client";
import { I18nProvider } from "./i18n/I18nProvider";
import { useAppStore } from "./state/appStore";
import "./styles.css";

useAppStore.getState().setUser(userStorage.get());

const queryClient = new QueryClient({
  defaultOptions: {
    queries: { retry: 2, staleTime: 1500 },
    mutations: { retry: false },
  },
});

ReactDOM.createRoot(document.getElementById("root")!).render(
  <QueryClientProvider client={queryClient}>
    <I18nProvider>
      <App />
    </I18nProvider>
  </QueryClientProvider>,
);
